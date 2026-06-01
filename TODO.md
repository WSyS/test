- [ ] Plan approved and implement AES-CBC fallback in components/wmbus_common/driver_brummerhoop.cc (only when t->dv_entries is empty)
- [ ] Remove the currently broken/duplicated AES fallback blocks and ensure file compiles

- [ ] Implement key derivation from YAML meter_id (16-byte AES key vs 4-byte meter id)
- [ ] Implement IV/ciphertext extraction from last_frame_ (Waterstarm EN 13757-3)
- [ ] Decrypt payload using AES_CBC_decrypt_buffer
- [ ] Extract total + total_backwards from decrypted bytes and setNumericValue
- [ ] Add logging to verify key/iv/ciphertext lengths and decrypted patterns (DIF/VIF)
- [ ] Build/compile and validate with provided log scenario

