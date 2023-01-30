#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#define ST_TOF_IOCTL_WFI 1

static struct miscdevice st_tof_miscdev;
static wait_queue_head_t wq;
static int intr_ready_flag = -1;

static int st_tof_dev_open(struct inode *inode, struct file *file)
{
	pr_debug("%s(%d)\n", __func__, __LINE__);
	return 0;
}

static int st_tof_dev_release(struct inode *inode, struct file *file)
{
	pr_debug("%s(%d)\n", __func__, __LINE__);
	return 0;

}

static long st_tof_dev_ioctl(struct file *file,
				 unsigned int cmd, unsigned long arg)
{

	/* pr_debug("st_tof_dev_ioctl : cmd = %u\n", cmd); */
	switch (cmd) {
	case ST_TOF_IOCTL_WFI:
		pr_debug("%s(%d)\n", __func__, __LINE__);
		wait_event_interruptible(wq, intr_ready_flag != 0);
		/* TODO :
		 * use wait_event_interruptible_timeout(timingBudget + margin)
		 */
		intr_ready_flag = 0;
		break;

	default:
		return -EINVAL;

	}
	return 0;
}

static const struct file_operations st_tof_dev_ranging_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = st_tof_dev_ioctl,
	.open = st_tof_dev_open,
	.release = st_tof_dev_release,
};

static irqreturn_t st_tof_intr_handler(int st_tof_irq_num, void *dev_id)
{
	intr_ready_flag = 1;
	wake_up_interruptible(&wq);
	return IRQ_HANDLED;
}

static int st_tof_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_desc *intr_gpiod;
	int irq, ret;

	init_waitqueue_head(&wq);

	intr_gpiod = devm_gpiod_get(dev, "intr", GPIOD_IN);
	if (IS_ERR(intr_gpiod))
		return dev_err_probe(dev, PTR_ERR(intr_gpiod), "failed to get intr-gpios\n");

	irq = gpiod_to_irq(intr_gpiod);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to map GPIO to IRQ\n");

	ret = devm_request_threaded_irq(dev,
									irq,
									NULL,
									st_tof_intr_handler,
									IRQF_TRIGGER_RISING | IRQF_ONESHOT,
									"st_tof_sensor",
									pdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "st_tof_dev";
	st_tof_miscdev.fops = &st_tof_dev_ranging_fops;

	ret = misc_register(&st_tof_miscdev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to create misc device\n");

	return 0;
}

static void st_tof_remove(struct platform_device *pdev)
{
	(void) pdev;

	misc_deregister(&st_tof_miscdev);
}

static const struct of_device_id st_tof_of_match[] = {
	{ .compatible = "st,tof" },
	{ },
};

MODULE_DEVICE_TABLE(of, st_tof_of_match);

static struct platform_driver st_tof_driver = {
	.driver = {
		.name = "st_tof",
		.of_match_table = st_tof_of_match
	},
	.probe = st_tof_probe,
	.remove = st_tof_remove,
};

module_platform_driver(st_tof_driver);

MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST VL53L1X sensor IT driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
