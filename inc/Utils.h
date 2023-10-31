/*
 * Utils.h
 *
 * Created: 9/5/2022 7:42:01 PM
 *  Author: Ahmed
 */ 


#ifndef UTILS_H_
#define UTILS_H_

#define SET_BIT(reg, bit)				( (reg) |= ( 1U << (bit) ) )
#define CLEAR_BIT(reg, bit)				( (reg) &= ~( 1U << (bit) ) )
#define TOGGLE_BIT(reg, bit)			( (reg) ^= ( 1U << (bit) ) )
#define READ_BIT(reg, bit)				( ( (reg) >> (bit) ) & 1U )
#define WRITE_BIT(reg, bit, val)		((reg) = ((reg) & (~(1U << (bit)))) | ((val) << (bit)))

#define WRITE_MASK_POS(reg, mask, pos, val) ((reg) = ((reg) & (~((mask) << (pos)))) | ((val) << (pos)))
#define WRITE_MASK(reg, mask, val)			((reg) = ((reg) & (~(mask))) | (val))

#define CLEAR_MASK(reg, mask)				((reg) &= ~(mask))
#define SET_MASK(reg, mask)					((reg) |= (mask))

#define READ_MASK_POS(reg, mask, pos)		(( (reg) >> (pos) ) & (mask))
#define READ_MASK(reg, mask)					((reg) & (mask))

#define Swap_32BIT_Endians(val) (((val) & 0x000000FFU) << 24U) | (((val) & 0x0000FF00U) << 8U) | (((val) & 0x00FF0000U) >> 8U) | (((val) & 0xFF000000U) >> 24U)
#define Swap_16BIT_Endians(val) ((((val) & 0x00FFU) << 8U) | (((val) & 0xFF00U) >> 8U))

#endif /* UTILS_H_ */
