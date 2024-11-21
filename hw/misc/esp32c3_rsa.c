/*
 * ESP32-C3 RSA accelerator
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/misc/esp32c3_rsa.h"


static const TypeInfo esp32c3_rsa_info = {
    .name = TYPE_ESP32C3_RSA,
    .parent = TYPE_ESP_RSA,
    .instance_size = sizeof(ESP32C3RsaState),
    .class_size = sizeof(ESP32C3RsaClass)
};

static void esp32c3_rsa_register_types(void)
{
    type_register_static(&esp32c3_rsa_info);
}

type_init(esp32c3_rsa_register_types)
