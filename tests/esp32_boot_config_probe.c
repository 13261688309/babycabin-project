#include "baby_care_config.h"

#if BABY_CARE_ESP32_CONFIGURE_ON_BOOT != EXPECTED_BOOT_POLICY
#error "Unexpected ESP32 boot policy"
#endif

int main(void)
{
    return 0;
}
