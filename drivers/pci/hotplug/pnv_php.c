// SPDX-License-Identifier: GPL-2.0+
/*
 * PCI Hotplug Driver for PowerPC PowerNV platform.
 *
 * Copyright Gavin Shan, IBM Corporation 2016.
 */

#include <linux/libfdt.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_hotplug.h>

#include <asm/opal.h>
#include <asm/pnv-pci.h>
#include <asm/ppc-pci.h>

#define DRIVER_VERSION	"0.1"
#define DRIVER_AUTHOR	"Gavin Shan, IBM Corporation"
#define DRIVER_DESC	"PowerPC PowerNV PCI Hotplug Driver"

struct pnv_php_event {
	bool			added;
	struct pnv_php_slot	*php_slot;
	struct work_struct	work;
};

static LIST_HEAD(pnv_php_slot_list);
static DEFINE_SPINLOCK(pnv_php_lock);

static void pnv_php_register(struct device_node *dn);
static void pnv_php_unregister_one(struct device_node *dn);
static void pnv_php_unregister(struct device_node *dn);

static void pnv_php_disable_irq(struct pnv_php_slot *php_slot,
				bool disable_device)
{
	struct pci_dev *pdev = php_slot->pdev;
	int irq = php_slot->irq;
	u16 ctrl;

	if (php_slot->irq > 0) {
		pcie_capability_read_word(pdev, PCI_EXP_SLTCTL, &ctrl);
		ctrl &= ~(PCI_EXP_SLTCTL_HPIE |
			  PCI_EXP_SLTCTL_PDCE |
			  PCI_EXP_SLTCTL_DLLSCE);
		pcie_capability_write_word(pdev, PCI_EXP_SLTCTL, ctrl);

		free_irq(php_slot->irq, php_slot);
		php_slot->irq = 0;
	}

	if (php_slot->wq) {
		destroy_workqueue(php_slot->wq);
		php_slot->wq = NULL;
	}

	if (disable_device || irq > 0) {
		if (pdev->msix_enabled)
			pci_disable_msix(pdev);
		else if (pdev->msi_enabled)
			pci_disable_msi(pdev);

		pci_disable_device(pdev);
	}
}

static void pnv_php_free_slot(struct kref *kref)
{
	struct pnv_php_slot *php_slot = container_of(kref,
					struct pnv_php_slot, kref);

	WARN_ON(!list_empty(&php_slot->children));
	pnv_php_disable_irq(php_slot, false);
	kfree(php_slot->name);
	kfree(php_slot);
}

static inline void pnv_php_put_slot(struct pnv_php_slot *php_slot)
{

	if (!php_slot)
		return;

	kref_put(&php_slot->kref, pnv_php_free_slot);
}

static struct pnv_php_slot *pnv_php_match(struct device_node *dn,
					  struct pnv_php_slot *php_slot)
{
	struct pnv_php_slot *target, *tmp;

	if (php_slot->dn == dn) {
		kref_get(&php_slot->kref);
		return php_slot;
	}

	list_for_each_entry(tmp, &php_slot->children, link) {
		target = pnv_php_match(dn, tmp);
		if (target)
			return target;
	}

	return NULL;
}

struct pnv_php_slot *pnv_php_find_slot(struct device_node *dn)
{
	struct pnv_php_slot *php_slot, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&pnv_php_lock, flags);
	list_for_each_entry(tmp, &pnv_php_slot_list, link) {
		php_slot = pnv_php_match(dn, tmp);
		if (php_slot) {
			spin_unlock_irqrestore(&pnv_php_lock, flags);
			return php_slot;
		}
	}
	spin_unlock_irqrestore(&pnv_php_lock, flags);

	return NULL;
}
EXPORT_SYMBOL_GPL(pnv_php_find_slot);

/*
 * Remove pdn for all children of the indicated device node.
 * The function should remove pdn in a depth-first manner.
 */
static void pnv_php_rmv_pdns(struct device_node *dn)
{
	struct device_node *child;

	for_each_child_of_node(dn, child) {
		pnv_php_rmv_pdns(child);

		pci_remove_device_node_info(child);
	}
}

/*
 * Detach all child nodes of the indicated device nodes. The
 * function should handle device nodes in depth-first manner.
 *
 * We should not invoke of_node_release() as the memory for
 * individual device node is part of large memory block. The
 * large block is allocated from memblock (system bootup) or
 * kmalloc() when unflattening the device tree by OF changeset.
 * We can not free the large block allocated from memblock. For
 * later case, it should be released at once.
 */
static void pnv_php_detach_device_nodes(struct device_node *parent)
{
	struct device_node *dn;
	int refcount;

	for_each_child_of_node(parent, dn) {
		pnv_php_detach_device_nodes(dn);

		of_node_put(dn);
		refcount = kref_read(&dn->kobj.kref);
		if (refcount != 1)
			pr_warn("Invalid refcount %d on <%pOF>\n",
				refcount, dn);

		of_detach_node(dn);
	}
}

static void pnv_php_rmv_devtree(struct pnv_php_slot *php_slot)
{
	pnv_php_rmv_pdns(php_slot->dn);

	/*
	 * Decrease the refcount if the device nodes were created
	 * through OF changeset before detaching them.
	 */
	if (php_slot->fdt)
		of_changeset_destroy(&php_slot->ocs);
	pnv_php_detach_device_nodes(php_slot->dn);

	if (php_slot->fdt) {
		kfree(php_slot->dt);
		kfree(php_slot->fdt);
		php_slot->dt        = NULL;
		php_slot->dn->child = NULL;
		php_slot->fdt       = NULL;
	}
}

/*
 * As the nodes in OF changeset are applied in reverse order, we
 * need revert the nodes in advance so that we have correct node
 * order after the changeset is applied.
 */
static void pnv_php_reverse_nodes(struct device_node *parent)
{
	struct device_node *child, *next;

	/* In-depth first */
	for_each_child_of_node(parent, child)
		pnv_php_reverse_nodes(child);

	/* Reverse the nodes in the child list */
	child = parent->child;
	parent->child = NULL;
	while (child) {
		next = child->sibling;

		child->sibling = parent->child;
		parent->child = child;
		child = next;
	}
}

static int pnv_php_populate_changeset(struct of_changeset *ocs,
				      struct device_node *dn)
{
	struct device_node *child;
	int ret = 0;

	for_each_child_of_node(dn, child) {
		ret = of_changeset_attach_node(ocs, child);
		if (ret) {
			of_node_put(child);
			break;
		}

		ret = pnv_php_populate_changeset(ocs, child);
		if (ret) {
			of_node_put(child);
			break;
		}
	}

	return ret;
}

static void *pnv_php_add_one_pdn(struct device_node *dn, void *data)
{
	struct pci_controller *hose = (struct pci_controller *)data;
	struct pci_dn *pdn;

	pdn = pci_add_device_node_info(hose, dn);
	if (!pdn)
		return ERR_PTR(-ENOMEM);

	return NULL;
}

static void pnv_php_add_pdns(struct pnv_php_slot *slot)
{
	struct pci_controller *hose = pci_bus_to_host(slot->bus);

	pci_traverse_device_nodes(slot->dn, pnv_php_add_one_pdn, hose);
}

static int pnv_php_add_devtree(struct pnv_php_slot *php_slot)
{
	void *fdt, *fdt1, *dt;
	int ret;

	/* We don't know the FDT blob size. We try to get it through
	 * maximal memory chunk and then copy it to another chunk that
	 * fits the real size.
	 */
	fdt1 = kzalloc(0x10000, GFP_KERNEL);
	if (!fdt1) {
		ret = -ENOMEM;
		goto out;
	}

	ret = pnv_pci_get_device_tree(php_slot->dn->phandle, fdt1, 0x10000);
	if (ret) {
		pci_warn(php_slot->pdev, "Error %d getting FDT blob\n", ret);
		goto free_fdt1;
	}

	fdt = kzalloc(fdt_totalsize(fdt1), GFP_KERNEL);
	if (!fdt) {
		ret = -ENOMEM;
		goto free_fdt1;
	}

	/* Unflatten device tree blob */
	memcpy(fdt, fdt1, fdt_totalsize(fdt1));
	dt = of_fdt_unflatten_tree(fdt, php_slot->dn, NULL);
	if (!dt) {
		ret = -EINVAL;
		pci_warn(php_slot->pdev, "Cannot unflatten FDT\n");
		goto free_fdt;
	}

	/* Initialize and apply the changeset */
	of_changeset_init(&php_slot->ocs);
	pnv_php_reverse_nodes(php_slot->dn);
	ret = pnv_php_populate_changeset(&php_slot->ocs, php_slot->dn);
	if (ret) {
		pnv_php_reverse_nodes(php_slot->dn);
		pci_warn(php_slot->pdev, "Error %d populating changeset\n",
			 ret);
		goto free_dt;
	}

	php_slot->dn->child = NULL;
	ret = of_changeset_apply(&php_slot->ocs);
	if (ret) {
		pci_warn(php_slot->pdev, "Error %d applying changeset\n", ret);
		goto destroy_changeset;
	}

	/* Add device node firmware data */
	pnv_php_add_pdns(php_slot);
	php_slot->fdt = fdt;
	php_slot->dt  = dt;
	kfree(fdt1);
	goto out;

destroy_changeset:
	of_changeset_destroy(&php_slot->ocs);
free_dt:
	kfree(dt);
	php_slot->dn->child = NULL;
free_fdt:
	kfree(fdt);
free_fdt1:
	kfree(fdt1);
out:
	return ret;
}

static int pnv_php_get_power_state(struct hotplug_slot *slot, u8 *state)
{
	struct pnv_php_slot *php_slot = slot->private;
	uint8_t power_state = OPAL_PCI_SLOT_POWER_ON;
	int ret;

	/*
	 * Retrieve power status from firmware. If we fail
	 * getting that, the power status fails back to
	 * be on.
	 */
	ret = pnv_pci_get_power_state(php_slot->id, &power_state);
	if (ret) {
		dev_warn(&php_slot->pdev->dev, "Error %d getting power status\n",
			 ret);
	} else {
		*state = power_state;
		slot->info->power_status = power_state;
	}

	return 0;
}

int pnv_php_set_slot_power_state(struct hotplug_slot *slot,
				 uint8_t state)
{
	struct pnv_php_slot *php_slot = slot->private;
	struct opal_msg msg;
	int ret;
	uint8_t power_status;

	/*
	 * Older firmware will return OPAL_BUSY if you attempt to
	 * set the state to itself (i.e. On -> On, or Off -> Off),
	 * so check for that.
	 */
	ret = pnv_php_get_power_state(slot, &power_status);
	if (ret > 0)
		return ret;
	if (power_status == state)
		return 0;

	ret = pnv_pci_set_power_state(php_slot->id, state, &msg);
	if (ret > 0) {
		if (be64_to_cpu(msg.params[1]) != php_slot->dn->phandle	||
		    be64_to_cpu(msg.params[2]) != state			||
		    be64_to_cpu(msg.params[3]) != OPAL_SUCCESS) {
			pci_warn(php_slot->pdev, "Wrong msg (%lld, %lld, %lld)\n",
				 be64_to_cpu(msg.params[1]),
				 be64_to_cpu(msg.params[2]),
				 be64_to_cpu(msg.params[3]));
			return -ENOMSG;
		}
	} else if (ret < 0) {
		pci_warn(php_slot->pdev, "Error %d powering %s\n",
			 ret, (state == OPAL_PCI_SLOT_POWER_ON) ? "on" : "off");
		return ret;
	}

	if (state == OPAL_PCI_SLOT_POWER_OFF || state == OPAL_PCI_SLOT_OFFLINE)
		pnv_php_rmv_devtree(php_slot);
	else
		ret = pnv_php_add_devtree(php_slot);

	return ret;
}
EXPORT_SYMBOL_GPL(pnv_php_set_slot_power_state);

static int pnv_php_get_adapter_state(struct hotplug_slot *slot, u8 *state)
{
	struct pnv_php_slot *php_slot = slot->private;
	uint8_t presence = OPAL_PCI_SLOT_EMPTY;
	int ret;

	/*
	 * Retrieve presence status from firmware. If we can't
	 * get that, it will fail back to be empty.
	 */
	ret = pnv_pci_get_presence_state(php_slot->id, &presence);
	if (ret >= 0) {
		*state = presence;
		slot->info->adapter_status = presence;
		ret = 0;
	} else {
		pci_warn(php_slot->pdev, "Error %d getting presence\n", ret);
	}

	return ret;
}

static int pnv_php_set_attention_state(struct hotplug_slot *slot, u8 state)
{
	/* FIXME: Make it real once firmware supports it */
	slot->info->attention_status = state;

	return 0;
}

static int pnv_php_enable(struct pnv_php_slot *php_slot, bool initial)
{
	struct hotplug_slot *slot = &php_slot->slot;
	uint8_t presence = OPAL_PCI_SLOT_EMPTY;
	uint8_t power = OPAL_PCI_SLOT_POWER_ON;
	int ret;

	if (php_slot->state != PNV_PHP_STATE_REGISTERED)
		return 0;

	/*
	 * If we don't have a presence detect then don't bother. If your
	 * system doesn't provide presence detect on a hotplug slot then
	 * your system is broken and you should fix it.
	 */
	ret = pnv_php_get_adapter_state(slot, &presence);
	dev_warn(&php_slot->pdev->dev, "presence %d (%d)\n", presence, ret);
	if (ret)
		return ret;
	if (presence == OPAL_PCI_SLOT_EMPTY) {
		dev_warn(&php_slot->pdev->dev, "unpopulated, slot empty\n");
		return 0;
	}

	ret = pnv_php_get_power_state(slot, &power);
	dev_warn(&php_slot->pdev->dev, "initial power state %d (%d)\n", power, ret);
	if (ret)
		return ret;


	/*
	 * If we're enabling a newly registered slot then we assume
	 * the DT is already populated if the slot is powered on.
	 *
	 * TODO: Check that the Linux DT matches the actual topology.
	 *       There might be interactions with EEH we need to be
	 *       concerned about here...
	 */
	if (power == OPAL_PCI_SLOT_POWER_ON) {
		if (!initial)
			dev_warn(&php_slot->pdev->dev, "ATTEMPTED TO POPULATE ALREADY POWERED ON SLOT!!!!\n");

		php_slot->state = PNV_PHP_STATE_POPULATED;

		return 0;
	}

	dev_warn(&php_slot->pdev->dev, "powering on...\n");
	ret = pnv_php_set_slot_power_state(slot, OPAL_PCI_SLOT_POWER_ON);
	if (ret) {
		dev_warn(&php_slot->pdev->dev, "after set_power_state %d (%d)\n", power, ret);
		return ret;
	}

	pci_lock_rescan_remove();
	pci_hp_add_devices(php_slot->bus);
	pci_unlock_rescan_remove();

	php_slot->state = PNV_PHP_STATE_POPULATED;
	dev_warn(&php_slot->pdev->dev, "populated!\n");

	pnv_php_register(php_slot->dn);

	return 0;
}

static int pnv_php_enable_slot(struct hotplug_slot *slot)
{
	struct pnv_php_slot *php_slot = container_of(slot,
						     struct pnv_php_slot, slot);

	return pnv_php_enable(php_slot, false);
}

static int pnv_php_disable_slot(struct hotplug_slot *slot)
{
	struct pnv_php_slot *php_slot = slot->private;
	int ret;
	uint8_t power = 0;

	/* Remove all devices behind the slot */
	pci_lock_rescan_remove();
	pci_hp_remove_devices(php_slot->bus);
	pci_unlock_rescan_remove();

	/* Detach the child hotpluggable slots */
	pnv_php_unregister(php_slot->dn);

	/* Notify firmware and remove device nodes */

	ret = pnv_php_get_power_state(slot, &power);
	dev_warn(&php_slot->pdev->dev, "%s: before: power = %d (%d)\n", __func__, power, ret);

	ret = pnv_php_set_slot_power_state(slot, OPAL_PCI_SLOT_POWER_OFF);
	dev_warn(&php_slot->pdev->dev, "%s: set_slot_power_state ret = %d\n", __func__, ret);
	

	ret = pnv_php_get_power_state(slot, &power);
	dev_warn(&php_slot->pdev->dev, "%s: after: power = %d (%d)\n", __func__, power, ret);

	php_slot->state = PNV_PHP_STATE_REGISTERED;
	return ret;
}

static struct hotplug_slot_ops php_slot_ops = {
	.get_power_status	= pnv_php_get_power_state,
	.get_adapter_status	= pnv_php_get_adapter_state,
	.set_attention_status	= pnv_php_set_attention_state,
	.enable_slot		= pnv_php_enable_slot,
	.disable_slot		= pnv_php_disable_slot,
};

static void pnv_php_release(struct pnv_php_slot *php_slot)
{
	unsigned long flags;

	/* Remove from global or child list */
	spin_lock_irqsave(&pnv_php_lock, flags);
	list_del(&php_slot->link);
	spin_unlock_irqrestore(&pnv_php_lock, flags);

	/* Detach from parent */
	pnv_php_put_slot(php_slot);
	pnv_php_put_slot(php_slot->parent);
}


/// XXX DUMB HACK
static int pnv_pci_get_slot_id2(struct device_node *np, uint64_t *id)
{
	struct device_node *parent = np;
	u32 bdfn;
	u64 phbid;
	int ret;

	ret = of_property_read_u32(np, "reg", &bdfn);
	if (ret)
		return -ENXIO;

	bdfn = ((bdfn & 0x00ffff00) >> 8);
	while ((parent = of_get_parent(parent))) {
		if (!PCI_DN(parent)) {
			of_node_put(parent);
			break;
		}

		if (!of_device_is_compatible(parent, "ibm,ioda2-phb") &&
			!of_device_is_compatible(parent, "ibm,ioda3-phb")) {
			of_node_put(parent);
			continue;
		}

		ret = of_property_read_u64(parent, "ibm,opal-phbid", &phbid);
		if (ret) {
			of_node_put(parent);
			return -ENXIO;
		}

		*id = PCI_SLOT_ID(phbid, bdfn);
		return 0;
	}

	return -ENODEV;
}
/// XXX DUMB HACK

static struct pnv_php_slot *pnv_php_alloc_slot(struct device_node *dn)
{
	struct pnv_php_slot *php_slot;
	struct pci_bus *bus;
	const char *label;
	uint64_t id;
	int ret;

	ret = of_property_read_string(dn, "ibm,slot-label", &label);
	if (ret)
		return NULL;

	if (pnv_pci_get_slot_id2(dn, &id))
		return NULL;

	bus = pci_find_bus_by_node(dn);
	if (!bus)
		return NULL;

	php_slot = kzalloc(sizeof(*php_slot), GFP_KERNEL);
	if (!php_slot)
		return NULL;

	php_slot->name = kstrdup(label, GFP_KERNEL);
	if (!php_slot->name) {
		kfree(php_slot);
		return NULL;
	}

	if (dn->child && PCI_DN(dn->child))
		php_slot->slot_no = PCI_SLOT(PCI_DN(dn->child)->devfn);
	else
		php_slot->slot_no = -1;   /* Placeholder slot */

	kref_init(&php_slot->kref);
	php_slot->state	                = PNV_PHP_STATE_INITIALIZED;
	php_slot->dn	                = dn;
	php_slot->pdev	                = bus->self;
	php_slot->bus	                = bus;
	php_slot->id	                = id;
	php_slot->slot.ops              = &php_slot_ops;
	php_slot->slot.info             = &php_slot->slot_info;
	php_slot->slot.private          = php_slot;

	INIT_LIST_HEAD(&php_slot->children);
	INIT_LIST_HEAD(&php_slot->link);

	return php_slot;
}

static int pnv_php_register_slot(struct pnv_php_slot *php_slot)
{
	struct pnv_php_slot *parent;
	struct device_node *dn = php_slot->dn;
	unsigned long flags;
	int ret;

	/* Check if the slot is registered or not */
	parent = pnv_php_find_slot(php_slot->dn);
	if (parent) {
		pnv_php_put_slot(parent);
		return -EEXIST;
	}

	/* Register PCI slot */
	ret = pci_hp_register(&php_slot->slot, php_slot->bus,
			      php_slot->slot_no, php_slot->name);
	if (ret) {
		pci_warn(php_slot->pdev, "Error %d registering slot\n", ret);
		return ret;
	}

	/* Attach to the parent's child list or global list */
	while ((dn = of_get_parent(dn))) {
		if (!PCI_DN(dn)) {
			of_node_put(dn);
			break;
		}

		parent = pnv_php_find_slot(dn);
		if (parent) {
			of_node_put(dn);
			break;
		}

		of_node_put(dn);
	}

	spin_lock_irqsave(&pnv_php_lock, flags);
	php_slot->parent = parent;
	if (parent)
		list_add_tail(&php_slot->link, &parent->children);
	else
		list_add_tail(&php_slot->link, &pnv_php_slot_list);
	spin_unlock_irqrestore(&pnv_php_lock, flags);

	php_slot->state = PNV_PHP_STATE_REGISTERED;
	return 0;
}

static int pnv_php_enable_msix(struct pnv_php_slot *php_slot)
{
	struct pci_dev *pdev = php_slot->pdev;
	struct msix_entry entry;
	int nr_entries, ret;
	u16 pcie_flag;

	/* Get total number of MSIx entries */
	nr_entries = pci_msix_vec_count(pdev);
	if (nr_entries < 0)
		return nr_entries;

	/* Check hotplug MSIx entry is in range */
	pcie_capability_read_word(pdev, PCI_EXP_FLAGS, &pcie_flag);
	entry.entry = (pcie_flag & PCI_EXP_FLAGS_IRQ) >> 9;
	if (entry.entry >= nr_entries)
		return -ERANGE;

	/* Enable MSIx */
	ret = pci_enable_msix_exact(pdev, &entry, 1);
	if (ret) {
		pci_warn(pdev, "Error %d enabling MSIx\n", ret);
		return ret;
	}

	return entry.vector;
}

static void pnv_php_event_handler(struct work_struct *work)
{
	struct pnv_php_event *event =
		container_of(work, struct pnv_php_event, work);
	struct pnv_php_slot *php_slot = event->php_slot;

	if (event->added)
		pnv_php_enable_slot(&php_slot->slot);
	else
		pnv_php_disable_slot(&php_slot->slot);

	kfree(event);
}

static irqreturn_t pnv_php_interrupt(int irq, void *data)
{
	struct pnv_php_slot *php_slot = data;
	struct pci_dev *pchild, *pdev = php_slot->pdev;
	struct eeh_dev *edev;
	struct eeh_pe *pe;
	struct pnv_php_event *event;
	u16 sts, lsts;
	u8 presence;
	bool added;
	unsigned long flags;
	int ret;

	pcie_capability_read_word(pdev, PCI_EXP_SLTSTA, &sts);

	pr_err("%s: pdc %d lldc %d\n", php_slot->name,
		!!(sts & PCI_EXP_SLTSTA_PDC), !!(sts & PCI_EXP_SLTSTA_DLLSC));

	sts &= (PCI_EXP_SLTSTA_PDC | PCI_EXP_SLTSTA_DLLSC);
	pcie_capability_write_word(pdev, PCI_EXP_SLTSTA, sts);

	if (sts & PCI_EXP_SLTSTA_DLLSC) {
		pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &lsts);
		added = !!(lsts & PCI_EXP_LNKSTA_DLLLA);
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d check LNKSTA %d\n", __FILE__, __LINE__, __func__, __LINE__, added);
	} else if (!(php_slot->flags & PNV_PHP_FLAG_BROKEN_PDC) &&
		   (sts & PCI_EXP_SLTSTA_PDC)) {
		ret = pnv_pci_get_presence_state(php_slot->id, &presence);
		if (ret) {
			pci_warn(pdev, "PCI slot [%s] error %d getting presence (0x%04x), to retry the operation.\n",
				 php_slot->name, ret, sts);
			return IRQ_HANDLED;
		}

		added = !!(presence == OPAL_PCI_SLOT_PRESENT);
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d check SLOT_PRESENT %d\n", __FILE__, __LINE__, __func__, __LINE__, added);
	} else {
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d in else\n", __FILE__, __LINE__, __func__, __LINE__);
		return IRQ_NONE;
	}

	/* Freeze the removed PE to avoid unexpected error reporting */
	if (!added) {
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d\n", __FILE__, __LINE__, __func__, __LINE__);
		pchild = list_first_entry_or_null(&php_slot->bus->devices,
						  struct pci_dev, bus_list);
	printk(KERN_WARNING "MINEDBG: %s:%d %s pchild %p\n", __FILE__, __LINE__, __func__, pchild);
		edev = pchild ? pci_dev_to_eeh_dev(pchild) : NULL;
	printk(KERN_WARNING "MINEDBG: %s:%d %s edev %p\n", __FILE__, __LINE__, __func__, edev);
		pe = edev ? edev->pe : NULL;
	printk(KERN_WARNING "MINEDBG: %s:%d %s pe %p\n", __FILE__, __LINE__, __func__, pe);
		if (pe) {
			eeh_serialize_lock(&flags);
			eeh_pe_state_mark(pe, EEH_PE_ISOLATED);
			eeh_serialize_unlock(flags);
			eeh_pe_set_option(pe, EEH_OPT_FREEZE_PE);
		}
	}

	/*
	 * The PE is left in frozen state if the event is missed. It's
	 * fine as the PCI devices (PE) aren't functional any more.
	 */
	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (!event) {
		pci_warn(pdev, "PCI slot [%s] missed hotplug event 0x%04x\n",
			 php_slot->name, sts);
		return IRQ_HANDLED;
	}

	pci_info(pdev, "PCI slot [%s] %s (IRQ: %d)\n",
		 php_slot->name, added ? "added" : "removed", irq);
	INIT_WORK(&event->work, pnv_php_event_handler);
	event->added = added;
	event->php_slot = php_slot;
	printk(KERN_WARNING "MINEDBG: %s:%d %s %d to queue work\n", __FILE__, __LINE__, __func__, __LINE__);
	queue_work(php_slot->wq, &event->work);

	return IRQ_HANDLED;
}

static void pnv_php_init_irq(struct pnv_php_slot *php_slot, int irq)
{
	struct pci_dev *pdev = php_slot->pdev;
	u32 broken_pdc = 0;
	u16 sts, ctrl;
	int ret;

	/* Allocate workqueue */
	php_slot->wq = alloc_workqueue("pciehp-%s", 0, 0, php_slot->name);
	if (!php_slot->wq) {
		pci_warn(pdev, "Cannot alloc workqueue\n");
		pnv_php_disable_irq(php_slot, true);
		return;
	}

	/* Check PDC (Presence Detection Change) is broken or not */
	ret = of_property_read_u32(php_slot->dn, "ibm,slot-broken-pdc",
				   &broken_pdc);
	if (!ret && broken_pdc)
		php_slot->flags |= PNV_PHP_FLAG_BROKEN_PDC;

	/* Clear pending interrupts */
	pcie_capability_read_word(pdev, PCI_EXP_SLTSTA, &sts);
	if (php_slot->flags & PNV_PHP_FLAG_BROKEN_PDC)
		sts |= PCI_EXP_SLTSTA_DLLSC;
	else
		sts |= (PCI_EXP_SLTSTA_PDC | PCI_EXP_SLTSTA_DLLSC);
	pcie_capability_write_word(pdev, PCI_EXP_SLTSTA, sts);

	/* Request the interrupt */
	ret = request_irq(irq, pnv_php_interrupt, IRQF_SHARED,
			  php_slot->name, php_slot);
	if (ret) {
		pnv_php_disable_irq(php_slot, true);
		pci_warn(pdev, "Error %d enabling IRQ %d\n", ret, irq);
		return;
	}

	/* Enable the interrupts */
	pcie_capability_read_word(pdev, PCI_EXP_SLTCTL, &ctrl);
	if (php_slot->flags & PNV_PHP_FLAG_BROKEN_PDC) {
		ctrl &= ~PCI_EXP_SLTCTL_PDCE;
		ctrl |= (PCI_EXP_SLTCTL_HPIE |
			 PCI_EXP_SLTCTL_DLLSCE);
	} else {
		ctrl |= (PCI_EXP_SLTCTL_HPIE |
			 PCI_EXP_SLTCTL_PDCE |
			 PCI_EXP_SLTCTL_DLLSCE);
	}
	pcie_capability_write_word(pdev, PCI_EXP_SLTCTL, ctrl);

	/* The interrupt is initialized successfully when @irq is valid */
	php_slot->irq = irq;
}

static void pnv_php_enable_irq(struct pnv_php_slot *php_slot)
{
	struct pci_dev *pdev = php_slot->pdev;
	int irq, ret;

	/*
	 * The MSI/MSIx interrupt might have been occupied by other
	 * drivers. Don't populate the surprise hotplug capability
	 * in that case.
	 */
	if (pci_dev_msi_enabled(pdev))
		return;

	ret = pci_enable_device(pdev);
	if (ret) {
		pci_warn(pdev, "Error %d enabling device\n", ret);
		return;
	}

	pci_set_master(pdev);

	/* Enable MSIx interrupt */
	irq = pnv_php_enable_msix(php_slot);
	if (irq > 0) {
		pnv_php_init_irq(php_slot, irq);
		return;
	}

	/*
	 * Use MSI if MSIx doesn't work. Fail back to legacy INTx
	 * if MSI doesn't work either
	 */
	ret = pci_enable_msi(pdev);
	if (!ret || pdev->irq) {
		irq = pdev->irq;
		pnv_php_init_irq(php_slot, irq);
	}
}

static int pnv_php_msg_hotplug_event(struct notifier_block *nb,
				     unsigned long msg_type, void *_msg);
static struct notifier_block pnv_php_msg_hotplug = {
	.notifier_call	= pnv_php_msg_hotplug_event,
	.next		= NULL,
	.priority	= 0,
};

struct pnv_php_slot *pnv_php_find_slot_by_id(uint64_t id)
{
	struct pnv_php_slot *tmp;
	unsigned long flags;

	spin_lock_irqsave(&pnv_php_lock, flags);
	list_for_each_entry(tmp, &pnv_php_slot_list, link) {
		if (tmp->id == id) {
			spin_unlock_irqrestore(&pnv_php_lock, flags);
			return tmp;
		}
	}
	spin_unlock_irqrestore(&pnv_php_lock, flags);

	return NULL;
}

static int pnv_php_msg_hotplug_event(struct notifier_block *nb,
				     unsigned long msg_type, void *_msg)
{
	struct opal_msg *msg = _msg;
	uint64_t id, added;
	struct pnv_php_slot *php_slot;
	struct pci_dev *pchild, *pdev;
	struct eeh_dev *edev;
	struct eeh_pe *pe;
	struct pnv_php_event *event;
	unsigned long flags;

	u16 sts, lsts;
	u8 presence;
	bool added_b;
	int ret;

	if (msg_type != OPAL_MSG_HOTPLUG)
		return 0;

	id = be64_to_cpu(msg->params[0]);
	added = be64_to_cpu(msg->params[1]);
	php_slot = pnv_php_find_slot_by_id(id);
	pdev = php_slot->pdev;

	printk(KERN_WARNING "MINEDBG: %s:%d %s id 0x%llx added 0x%llx\n",
			__FILE__, __LINE__, __func__, id, added);
	printk(KERN_WARNING "MINEDBG: %s:%d %s php_slot %p, pdev %p\n",
			__FILE__, __LINE__, __func__, php_slot, pdev);

	pcie_capability_read_word(pdev, PCI_EXP_SLTSTA, &sts);

	pr_err("%s: pdc %d lldc %d\n", php_slot->name,
		!!(sts & PCI_EXP_SLTSTA_PDC), !!(sts & PCI_EXP_SLTSTA_DLLSC));

	sts &= (PCI_EXP_SLTSTA_PDC | PCI_EXP_SLTSTA_DLLSC);
	pcie_capability_write_word(pdev, PCI_EXP_SLTSTA, sts);

	if (sts & PCI_EXP_SLTSTA_DLLSC) {
		pcie_capability_read_word(pdev, PCI_EXP_LNKSTA, &lsts);
		added_b = !!(lsts & PCI_EXP_LNKSTA_DLLLA);
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d check LNKSTA %d\n", __FILE__, __LINE__, __func__, __LINE__, added_b);
	} else if (!(php_slot->flags & PNV_PHP_FLAG_BROKEN_PDC) &&
		   (sts & PCI_EXP_SLTSTA_PDC)) {
		ret = pnv_pci_get_presence_state(php_slot->id, &presence);
		if (ret) {
			pci_warn(pdev, "PCI slot [%s] error %d getting presence (0x%04x), to retry the operation.\n",
				 php_slot->name, ret, sts);
			return 0;
		}

		added_b = !!(presence == OPAL_PCI_SLOT_PRESENT);
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d check SLOT_PRESENT %d\n", __FILE__, __LINE__, __func__, __LINE__, added_b);
	} else {
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d in else\n", __FILE__, __LINE__, __func__, __LINE__);
	}

	/* Freeze the removed PE to avoid unexpected error reporting */
	if (!added) {
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d\n", __FILE__, __LINE__, __func__, __LINE__);
		pchild = list_first_entry_or_null(&php_slot->bus->devices,
						  struct pci_dev, bus_list);
	printk(KERN_WARNING "MINEDBG: %s:%d %s pchild %p\n", __FILE__, __LINE__, __func__, pchild);
		edev = pchild ? pci_dev_to_eeh_dev(pchild) : NULL;
	printk(KERN_WARNING "MINEDBG: %s:%d %s edev %p\n", __FILE__, __LINE__, __func__, edev);
		pe = edev ? edev->pe : NULL;
	printk(KERN_WARNING "MINEDBG: %s:%d %s pe %p\n", __FILE__, __LINE__, __func__, pe);
		if (pe) {
		printk(KERN_WARNING "MINEDBG: %s:%d %s %d in pe\n", __FILE__, __LINE__, __func__, __LINE__);
			eeh_serialize_lock(&flags);
			eeh_pe_state_mark(pe, EEH_PE_ISOLATED);
			eeh_serialize_unlock(flags);
			eeh_pe_set_option(pe, EEH_OPT_FREEZE_PE);
		}
	}

	printk(KERN_WARNING "MINEDBG: %s:%d %s %d to kzalloc\n", __FILE__, __LINE__, __func__, __LINE__);
	/*
	 * The PE is left in frozen state if the event is missed. It's
	 * fine as the PCI devices (PE) aren't functional any more.
	 */
	event = kzalloc(sizeof(*event), GFP_ATOMIC);
	if (!event) {
		pci_warn(pdev, "PCI slot [%s] missed hotplug event %llu\n",
			 php_slot->name, added);
		return 0;
	}

	printk(KERN_WARNING "MINEDBG: %s:%d %s %d to init work\n", __FILE__, __LINE__, __func__, __LINE__);

	pci_info(pdev, "PCI slot [%s] %s\n",
		 php_slot->name, added ? "added" : "removed");
	INIT_WORK(&event->work, pnv_php_event_handler);
	event->added = added;
	event->php_slot = php_slot;
	printk(KERN_WARNING "MINEDBG: %s:%d %s %d to queue work\n", __FILE__, __LINE__, __func__, __LINE__);
	queue_work(php_slot->wq, &event->work);
	return 0;
}

static int pnv_php_register_one(struct device_node *dn)
{
	static bool is_message_subscribed = false;
	struct pnv_php_slot *php_slot;
	u32 prop32;
	int ret;

	pr_err("try register slot %pOF...\n", dn);

	/* Check if it's hotpluggable slot */
	ret = of_property_read_u32(dn, "ibm,slot-pluggable", &prop32);
	if (ret || !prop32) {
		pr_err("\t failed due to missing slot-pluggable\n");
		return -ENXIO;
	}

	/* when is this set? for all PHB direct slots? */
	ret = of_property_read_u32(dn, "ibm,reset-by-firmware", &prop32);
	if (ret || !prop32) {
		pr_err("\t failed due to reset-by-firmware\n");
		return -ENXIO;
	}

	php_slot = pnv_php_alloc_slot(dn);
	if (!php_slot) {
		pr_err("slot alloc failed!\n");
		return -ENODEV;
	}

	ret = pnv_php_register_slot(php_slot);
	if (ret) {
		pr_err("register failed!\n");
		goto free_slot;
	}

	/*
	 * Failing to enable the slot is non-fatal. We still want to keep around
	 * slot in the registered state so we can try online it later.
	 */
	pnv_php_enable(php_slot, true);

	ret = of_property_read_u32(dn, "ibm,use-hotplug-event", &prop32);
	if (!ret && prop32) {
		printk(KERN_WARNING "MINEDBG: %s:%d %s subscribe hotplug event on slot %p\n", __FILE__, __LINE__, __func__, php_slot);
		// TODO: FIXME: this needs to check if it's root complex, and only subscribe opal message when it is
		// And likely it should only register once
		if (!is_message_subscribed) {
			opal_message_notifier_register(OPAL_MSG_HOTPLUG, &pnv_php_msg_hotplug);
			is_message_subscribed = true;
		}
		/* Allocate workqueue */
		php_slot->wq = alloc_workqueue("pciehp-%s", 0, 0, php_slot->name);
		if (!php_slot->wq) {
			pci_warn(php_slot->pdev, "Cannot alloc workqueue\n");
			goto free_slot;
		}
		return 0;
	}

	/* Enable interrupt if the slot supports surprise hotplug */
	ret = of_property_read_u32(dn, "ibm,slot-surprise-pluggable", &prop32);
	if (!ret && prop32)
		pnv_php_enable_irq(php_slot);

	return 0;

free_slot:
	pnv_php_put_slot(php_slot);
	return ret;
}

static void pnv_php_register(struct device_node *dn)
{
	struct device_node *child;

	/*
	 * The parent slots should be registered before their
	 * child slots.
	 */
	for_each_child_of_node(dn, child) {
		pnv_php_register_one(child);
		pnv_php_register(child);
	}
}

static void pnv_php_unregister_one(struct device_node *dn)
{
	struct pnv_php_slot *php_slot;

	php_slot = pnv_php_find_slot(dn);
	if (!php_slot)
		return;

	php_slot->state = PNV_PHP_STATE_OFFLINE;
	pci_hp_deregister(&php_slot->slot);
	pnv_php_release(php_slot);
	pnv_php_put_slot(php_slot);
}

static void pnv_php_unregister(struct device_node *dn)
{
	struct device_node *child;

	/* The child slots should go before their parent slots */
	for_each_child_of_node(dn, child) {
		pnv_php_unregister(child);
		pnv_php_unregister_one(child);
	}
}

static int __init pnv_php_init(void)
{
	struct device_node *dn;

	pr_info(DRIVER_DESC " version: " DRIVER_VERSION "\n");

	for_each_compatible_node(dn, NULL, "ibm,ioda2-phb") {
		pr_err("foudn ioda2-phb %pOF...\n", dn);
		pnv_php_register(dn);
	}

	for_each_compatible_node(dn, NULL, "ibm,ioda3-phb") {
		pr_err("foudn ioda3-phb %pOF...\n", dn);
		pnv_php_register(dn);
	}

	return 0;
}

static void __exit pnv_php_exit(void)
{
	struct device_node *dn;

	for_each_compatible_node(dn, NULL, "ibm,ioda2-phb")
		pnv_php_unregister(dn);

	for_each_compatible_node(dn, NULL, "ibm,ioda3-phb")
		pnv_php_unregister(dn);
}

module_init(pnv_php_init);
module_exit(pnv_php_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
