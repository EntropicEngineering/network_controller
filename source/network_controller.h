#ifdef __cplusplus
extern "C" {
#endif
int
normal_read_operation(uint8_t page, uint8_t oset
                      , uint8_t *result, size_t len);
int
normal_write_operation(uint8_t page, uint8_t oset
                       , uint8_t *buf, size_t len);
void
bcm_init_spi(void);

#ifdef __cplusplus
} // extern "C"
#endif
