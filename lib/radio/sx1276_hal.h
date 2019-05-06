#ifndef __SX1276_HAL_H__
#define __SX1276_HAL_H__

#include <stdint.h>

typedef enum
{
    RADIO_RESET_OFF,
    RADIO_RESET_ON,
} radio_reset_state_t;

/*!
 * \brief Initializes the radio interface I/Os
 */
void sx1276_init_io(void);

/*!
 * \brief Set the radio reset pin state
 * 
 * \param state New reset pin state
 */
void sx1276_set_reset(uint8_t state);

/*!
 * \brief Writes the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [IN]: data New register value
 */
void sx1276_write(uint8_t addr, uint8_t data);

/*!
 * \brief Reads the radio register at the specified address
 *
 * \param [IN]: addr Register address
 * \param [OUT]: data Register value
 */
void sx1276_read(uint8_t addr, uint8_t *data);

/*!
 * \brief Writes multiple radio registers starting at address
 *
 * \param [IN] addr   First Radio register address
 * \param [IN] buffer Buffer containing the new register's values
 * \param [IN] size   Number of registers to be written
 */
void sx1276_write_buffer(uint8_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads multiple radio registers starting at address
 *
 * \param [IN] addr First Radio register address
 * \param [OUT] buffer Buffer where to copy the registers data
 * \param [IN] size Number of registers to be read
 */
void sx1276_read_buffer(uint8_t addr, uint8_t *buffer, uint8_t size);

/*!
 * \brief Writes the buffer contents to the radio FIFO
 *
 * \param [IN] buffer Buffer containing data to be put on the FIFO.
 * \param [IN] size Number of bytes to be written to the FIFO
 */
void sx1276_write_fifo(uint8_t *buffer, uint8_t size);

/*!
 * \brief Reads the contents of the radio FIFO
 *
 * \param [OUT] buffer Buffer where to copy the FIFO read data.
 * \param [IN] size Number of bytes to be read from the FIFO
 */
void sx1276_read_fifo(uint8_t *buffer, uint8_t size);

/*!
 * \brief Gets the SX1276 DIO0 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t sx1276_read_dio0(void);

/*!
 * \brief Gets the SX1276 DIO1 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t sx1276_read_dio1(void);

/*!
 * \brief Gets the SX1276 DIO2 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t sx1276_read_dio2(void);

/*!
 * \brief Gets the SX1276 DIO3 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t sx1276_read_dio3(void);

/*!
 * \brief Gets the SX1276 DIO4 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t sx1276_read_dio4(void);

/*!
 * \brief Gets the SX1276 DIO5 hardware pin status
 *
 * \retval status Current hardware pin status [1, 0]
 */
uint8_t sx1276_read_dio5(void);

#endif //__SX1276_HAL_H__