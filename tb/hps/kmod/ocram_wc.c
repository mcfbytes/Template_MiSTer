// SPDX-License-Identifier: GPL-2.0
/*
 * ocram_wc: OCRAM M7 test module (not for shipping). One misc device, /dev/ocram_wc, whose mmap maps the HPS
 * on-chip RAM (0xFFFF0000, 64 KB) WRITE-COMBINED (pgprot_writecombine: ARMv7 Normal non-cacheable, bufferable),
 * the map /dev/mem cannot give for a non-RAM page. One ioctl, OCRAM_WC_WMB, runs the kernel's wmb(): on this
 * kernel (ARM_HEAVY_MB, OUTER_CACHE_SYNC) a dsb st followed by the PL310's cache sync. Nothing else.
 */
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/io.h>

#define OCRAM_PHYS 0xFFFF0000UL
#define OCRAM_LEN  0x10000UL
#define OCRAM_WC_WMB _IO('o', 1)

static int ocram_wc_mmap(struct file *f, struct vm_area_struct *vma)
{
	unsigned long len = vma->vm_end - vma->vm_start;

	if (vma->vm_pgoff != 0 || len > OCRAM_LEN)
		return -EINVAL;
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, OCRAM_PHYS >> PAGE_SHIFT, len, vma->vm_page_prot);
}

static long ocram_wc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	if (cmd != OCRAM_WC_WMB)
		return -ENOTTY;
	wmb();
	return 0;
}

static const struct file_operations ocram_wc_fops = {
	.owner = THIS_MODULE,
	.mmap = ocram_wc_mmap,
	.unlocked_ioctl = ocram_wc_ioctl,
};

static struct miscdevice ocram_wc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ocram_wc",
	.fops = &ocram_wc_fops,
	.mode = 0600,
};

module_misc_device(ocram_wc_dev);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OCRAM M7 test: a write-combined map of the HPS on-chip RAM");
