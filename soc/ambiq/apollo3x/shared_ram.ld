/*
 * Copyright (c) 2023 Ambiq Micro Inc. <www.ambiq.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if CONFIG_DT_HAS_AMBIQ_MSPI_CONTROLLER_ENABLED
    SECTION_PROLOGUE (mspi_buff, CONFIG_SRAM_BASE_ADDRESS + CONFIG_MSPI_BUFF_RAM_LOCATION (NOLOAD),)
    {
        __mspi_buff_start = .;
        KEEP(*(SORT_BY_NAME(".mspi_buff*")))
        __mspi_buff_end = .;
    } GROUP_LINK_IN(RAMABLE_REGION)
#endif
