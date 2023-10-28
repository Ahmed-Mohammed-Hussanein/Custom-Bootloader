/*
 * Utils.h
 *
 * Created: 9/5/2022 7:42:01 PM
 *  Author: Ahmed
 */ 


#ifndef UTILS_H_
#define UTILS_H_

#define SET_BIT(reg, bit)				( (reg) |= ( 1 << (bit) ) )
#define CLEAR_BIT(reg, bit)				( (reg) &= ~( 1 << (bit) ) )
#define TOGGLE_BIT(reg, bit)			( (reg) ^= ( 1 << (bit) ) )
#define READ_BIT(reg, bit)				( ( (reg) >> (bit) ) & 1 )
#define WRITE_BIT(reg, bit, val)		((reg) = ((reg) & (~(1 << (bit)))) | ((val) << (bit)))

#define WRITE_MASK_POS(reg, mask, pos, val) ((reg) = ((reg) & (~((mask) << (pos)))) | ((val) << (pos)))
#define WRITE_MASK(reg, mask, val)			((reg) = ((reg) & (~(mask))) | (val))

#define CLEAR_MASK(reg, mask)				((reg) &= ~(mask))
#define SET_MASK(reg, mask)					((reg) |= (mask))

#define READ_MASK_POS(reg, mask, pos)		(( (reg) >> (pos) ) & (mask))
#define READ_MASK(reg, mask)					((reg) & (mask))

#define Swap_32BIT_Endians(val) ((val & 0x000000FF) << 24) | ((val & 0x0000FF00) << 8) | ((val & 0x00FF0000) >> 8) | ((val & 0xFF000000) >> 24)
#define Swap_16BIT_Endians(val) ((val & 0x00FF) << 8) | ((val & 0xFF00) >> 8))

#endif /* UTILS_H_ */
