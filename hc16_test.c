#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/hid.h>

struct usb_device_id id_table[] =
{
    {USB_DEVICE(0x256c, 0x006d)},
    {0}
};

MODULE_DEVICE_TABLE (usb, id_table);

void hc16_disconnect(struct usb_interface *interface)
{
    printk("dt_disconnect called\n");
}

int hc16_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    printk("dt_probe called\n");
    return 0;
}

static struct usb_driver hc16_driver =
{
    .name = "Hello HC16",
    .probe = hc16_probe,
    .disconnect = hc16_disconnect,
    .id_table = id_table
};

static int __init hc16_init(void)
{
    printk("hc16_init try");
    //0 means success
    int error = usb_register(&hc16_driver);
    if(error)
    {
        printk("hc16_init failed\n");
        return error;
    }

    printk("hc16_init done");

    return 0;
}

static void __exit hc16_exit(void)
{
    //void
    usb_deregister(&hc16_driver);
}

module_init(hc16_init);
module_exit(hc16_exit);

MODULE_LICENSE("GPL");