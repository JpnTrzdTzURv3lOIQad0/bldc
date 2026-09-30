/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"
#include "hw.h"

static void gpio_init(stm32_gpio_t *gpiop,
                      uint32_t moder,
                      uint32_t otyper,
                      uint32_t ospeedr,
                      uint32_t pupdr,
                      uint32_t odr,
                      uint32_t afrl,
                      uint32_t afrh) {
  gpiop->OTYPER = otyper;
  gpiop->OSPEEDR = ospeedr;
  gpiop->PUPDR = pupdr;
  gpiop->ODR = odr;
  gpiop->AFRL = afrl;
  gpiop->AFRH = afrh;
  gpiop->MODER = moder;
}

static void gpio_init_all(void) {
  rccResetAHB1(STM32_GPIO_EN_MASK);
  rccEnableAHB1(STM32_GPIO_EN_MASK, TRUE);
#if STM32_HAS_GPIOA
  gpio_init(GPIOA, VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR,
            VAL_GPIOA_PUPDR, VAL_GPIOA_ODR, VAL_GPIOA_AFRL, VAL_GPIOA_AFRH);
#endif
#if STM32_HAS_GPIOB
  gpio_init(GPIOB, VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR,
            VAL_GPIOB_PUPDR, VAL_GPIOB_ODR, VAL_GPIOB_AFRL, VAL_GPIOB_AFRH);
#endif
#if STM32_HAS_GPIOC
  gpio_init(GPIOC, VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR,
            VAL_GPIOC_PUPDR, VAL_GPIOC_ODR, VAL_GPIOC_AFRL, VAL_GPIOC_AFRH);
#endif
#if STM32_HAS_GPIOD
  gpio_init(GPIOD, VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR,
            VAL_GPIOD_PUPDR, VAL_GPIOD_ODR, VAL_GPIOD_AFRL, VAL_GPIOD_AFRH);
#endif
#if STM32_HAS_GPIOE
  gpio_init(GPIOE, VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR,
            VAL_GPIOE_PUPDR, VAL_GPIOE_ODR, VAL_GPIOE_AFRL, VAL_GPIOE_AFRH);
#endif
#if STM32_HAS_GPIOF
  gpio_init(GPIOF, VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR,
            VAL_GPIOF_PUPDR, VAL_GPIOF_ODR, VAL_GPIOF_AFRL, VAL_GPIOF_AFRH);
#endif
#if STM32_HAS_GPIOG
  gpio_init(GPIOG, VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR,
            VAL_GPIOG_PUPDR, VAL_GPIOG_ODR, VAL_GPIOG_AFRL, VAL_GPIOG_AFRH);
#endif
#if STM32_HAS_GPIOH
  gpio_init(GPIOH, VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR,
            VAL_GPIOH_PUPDR, VAL_GPIOH_ODR, VAL_GPIOH_AFRL, VAL_GPIOH_AFRH);
#endif
#if STM32_HAS_GPIOI
  gpio_init(GPIOI, VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR,
            VAL_GPIOI_PUPDR, VAL_GPIOI_ODR, VAL_GPIOI_AFRL, VAL_GPIOI_AFRH);
#endif
}

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {
  HW_VERY_EARLY_INIT();
  stm32_clock_init();
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
/**
 * @brief   SDC card detection.
 */
bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {

  (void)sdcp;
  /* TODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   SDC card write protection detection.
 */
bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

  (void)sdcp;
  /* TODO: Fill the implementation.*/
  return false;
}
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
/**
 * @brief   MMC_SPI card detection.
 */
bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return true;
}

/**
 * @brief   MMC_SPI card write protection detection.
 */
bool mmc_lld_is_write_protected(MMCDriver *mmcp) {

  (void)mmcp;
  /* TODO: Fill the implementation.*/
  return false;
}
#endif

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
  gpio_init_all();
}
