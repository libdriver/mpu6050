/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      mutex.c
 * @brief     mutex source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2022-11-11
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2022/11/11  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "mutex.h"

static volatile uint8_t gs_locked = 0;                 /**< mutex locked flag */
static volatile uint32_t gs_int_locked_cnt = 0;        /**< mutex interrupt locked counter */
static uint8_t (*gs_irq)(void) = NULL;                 /**< mutex irq */

/**
 * @brief  mutex lock
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t mutex_lock(void)
{
    /* check if find the interrupt mutex */
    if (gs_int_locked_cnt != 0)
    {
        /* if having the irq */
        if (gs_irq != NULL)
        {
            /* run the callback */
            gs_irq();
        }
        
        /* clear the interrupt counter */
        gs_int_locked_cnt = 0;
    }
    
    /* flag locked */
    gs_locked = 1;
    
    return 0;
}

/**
 * @brief  mutex unlock
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t mutex_unlock(void)
{
    /* check if find the interrupt mutex */
    if (gs_int_locked_cnt != 0)
    {
        /* if having the irq */
        if (gs_irq != NULL)
        {
            /* run the callback */
            gs_irq();
        }
        
        /* clear the interrupt counter */
        gs_int_locked_cnt = 0;
    }
    
    /* flag unlocked */
    gs_locked = 0;
    
    return 0;
}

/**
 * @brief     mutex irq
 * @param[in] *irq pointer to an interrupt funtion
 * @note      none
 */
void mutex_irq(uint8_t (*irq)(void))
{
    /* if not locked */
    if (gs_locked == 0)
    {
        /* if having the irq */
        if (irq != NULL)
        {
            /* run the callback */
            irq();
        }
    }
    else
    {
        /* set the irq callback */
        gs_irq = irq;
        
        /* interrupt mutex counter increment */
        gs_int_locked_cnt++;
    }
}
