#include"uart_packets.h"

/*
* UART Packets is a set of utility tools that aims to abstract away the encapsulation problem of 
* UART. 
*/


static const char* TAG = "UART Packets";

/**
 * @brief Returns a void* to the end of the packet.
 * @param wrapper_ptr the pointer to operate on
 * @return void*, the end of the packet buffer
*/
static void* uart_packets_get_end_of_packet_buffer(uart_packets_packet_wrapper_t* wrapper_ptr){
    char* packet_end_ptr = wrapper_ptr->packet_ptr;
    // for a buf of size n, the end element is at the n-1th index. Do not forget packet_end_ptr itself is the first element. 
    packet_end_ptr += wrapper_ptr->packet_len -1;
    return packet_end_ptr;
}

/**
 * @brief Serializes a wrapper objects to a byte array into the pointer supplied. 
 * @details The memory of the dest ptr will not be owned by the wrapper object. 
 * @param wrapper_ptr A pointer to the wrapper which will be serialized
 * @param dest Where the serialized bytes will be written to 
 * @returns esp_err_t, operation state
*/
esp_err_t uart_packets_serialize_wrapper_to_bytes(uart_packets_packet_wrapper_t *wrapper_ptr, void *dest)
{
    // null check 
    if(wrapper_ptr == NULL || dest == NULL){
        return ESP_ERR_NOT_FOUND;
    }

    memcpy(dest,wrapper_ptr->packet_ptr,wrapper_ptr->packet_len);
    return ESP_OK;
}

/**
 * @brief not implemented yet :)
*/
uart_packets_packet_wrapper_t uart_packets_deserialize_bytes_to_new_wrapper(uart_packets_packet_conf_t config, void *src, size_t len)
{
    uart_packets_packet_wrapper_t ret  = {0};
    return ret;
}

/**
 * @brief Deserializes a byte array into an existing wrapper, without allocating memory. Old wrapper will be overwritten, with the configuration.
 * @details This will fail if the new packet cannot be held by the existing buffer of the wrapper. Function does not allocate the packet, even if it is a dynamic one.
 * @param config The expected packet configuration of the to be read packet
 * @param src The packet bytes to deserialize from
 * @param wrapper_ptr The wrapper to deserialize into 
 * @returns esp_err_t operation state
*/
esp_err_t uart_packets_deserialize_bytes_to_existing_wrapper(uart_packets_packet_conf_t config, void *src, uart_packets_packet_wrapper_t *wrapper_ptr)
{
    if(src == NULL || wrapper_ptr == NULL ){
        return ESP_ERR_NOT_FOUND;
    }
    // overwrite the old config
    wrapper_ptr->wrapper_config = config;
    // get the actual packet size
    uint8_t actual_packet_size;
    uint8_t* packet_size_ptr = (uint8_t*) src;
    packet_size_ptr += config.packet_size_offset;
    actual_packet_size = *packet_size_ptr;

    if(actual_packet_size != wrapper_ptr->packet_len){
        ESP_LOGE(TAG,"Deserialize into existing packet failed, lengts do not match. Expected:%u,actual:%u",wrapper_ptr->packet_len,actual_packet_size);
        return ESP_ERR_INVALID_ARG;
    }
    // copy the new data in 
    memcpy(wrapper_ptr->packet_ptr,src,wrapper_ptr->packet_len);
    // sanity check the termination characters 
    char* packet_term_start_ptr = uart_packets_get_end_of_packet_buffer(wrapper_ptr);
    packet_term_start_ptr -= config.termination_len - 1; 
    // check term chars one by one 
    size_t observed_char_count = 0;
    for (size_t i = 0; i < config.termination_len; i++)
    {
        if(packet_term_start_ptr[i] == config.termination_chr){
            observed_char_count++;
        }
    }
    // check if the actual meets the expected
    if(observed_char_count != config.termination_len){
        ESP_LOGE(TAG,"Deserializing failed, term len do not match. Expected:%u,Actual:%u",config.termination_len,observed_char_count);
        return ESP_ERR_INVALID_RESPONSE;
    }
    //done
    return ESP_OK;
}

/**
 * @brief Create an empty packet by dynamic allocation. 
 * @details Always null-check the returned pointer as dynamic allocation can fail.
 * @param config The packet configuration 
 * @param packet_len the length of the empty packet, including termination chars
 * @returns uart_packets_packet_wrapper_t, the packet that was created.
*/
uart_packets_packet_wrapper_t uart_packets_create_empty_packet(uart_packets_packet_conf_t config, size_t packet_len)
{
    uart_packets_packet_wrapper_t new_wrapper = {0};
    void* packet_buf = malloc(packet_len);
    if(packet_buf == NULL){
        return new_wrapper;        
    }
    uart_packets_static_create_empty_packet(config,packet_buf,packet_len,&new_wrapper);
    new_wrapper.is_dynamic_allocated = true;
    return new_wrapper;
}

/**
 * @brief Create an empty packet by emplacing the contents into the void*. The user is responsible for the memory lifetime of the wrapper.  
 * @details The supplied packet_len must be correct or undefined behaviour may occur. Do not call free on this type of packet.
 * @param config The packet configuration 
 * @param packet_len the length of the empty packet, including termination chars
 * @returns uart_packets_packet_wrapper_t, the packet that was created.
*/
esp_err_t uart_packets_static_create_empty_packet(uart_packets_packet_conf_t config, void* packet_buf, uint8_t packet_len, uart_packets_packet_wrapper_t *wrapper_ptr)
{
    if(packet_buf == NULL || wrapper_ptr == NULL){
        return ESP_ERR_NOT_FOUND;
    }
    wrapper_ptr->packet_len = packet_len;
    wrapper_ptr->packet_ptr = packet_buf;
    wrapper_ptr->is_dynamic_allocated = false;
    wrapper_ptr->wrapper_config = config;
    // set the packet size 
    uart_packets_set_header_data_from_wrapper(wrapper_ptr, config.packet_size_offset, &wrapper_ptr->packet_len, sizeof(uint8_t));
    size_t term_char_len = config.termination_len;
    // get the packet end
    char* packet_term_start_ptr = uart_packets_get_end_of_packet_buffer(wrapper_ptr);
    // turn it into term start
    packet_term_start_ptr -= config.termination_len - 1;
    
    memset(packet_term_start_ptr,config.termination_chr,term_char_len);
    return ESP_OK;
}

/**
 * @brief Destroys the packet if it is dynamically allocated.  
 * @param uart_packets_packet_wrapper_t the packet to be deleted.
 * @returns esp_err_t, the state of the operation
*/
esp_err_t uart_packets_destroy_packet(uart_packets_packet_wrapper_t *wrapper_ptr)
{
    if(wrapper_ptr == NULL){
        return ESP_ERR_NOT_FOUND;
    }
    if(!wrapper_ptr->is_dynamic_allocated){
        ESP_LOGE(TAG,"You cannot delete a static packet !!");
        return ESP_ERR_INVALID_ARG;
    }
    free(wrapper_ptr->packet_ptr);
    uart_packets_packet_conf_t empty_config = {0};
    wrapper_ptr->wrapper_config = empty_config;
    return ESP_OK;
}

/**
 * @brief Extract data from the header of the packet
 * @details this function will fail if the request goes out of header bounds defined by configuration 
 * @param wrapper_ptr The packet wrapper to extract header bytes from
 * @param offset The offset in the header of the packet to read from, so the read will start from this byte instead of 0.
 * @param dest where the read header data will be copied to.
 * @param len the expected read length of the data from the header.
 * @returns esp_err_t the operation state
*/
esp_err_t uart_packets_get_header_data_from_wrapper(uart_packets_packet_wrapper_t *wrapper_ptr, size_t offset, void *dest, size_t len)
{
    // null check
    if(wrapper_ptr == NULL || dest == NULL ){
        return ESP_ERR_NOT_FOUND;
    }
    // see if offset goes out of bounds
    if ( ((offset + len) > wrapper_ptr->wrapper_config.header_len )){
        return ESP_ERR_INVALID_ARG;
    }
    char* packet_char_ptr = (char*) wrapper_ptr->packet_ptr;
    // do the offset on the packet
    packet_char_ptr += offset;
    memcpy(dest,packet_char_ptr,len);
    return ESP_OK;
}

/**
 * @brief Extract data from the body of the packet
 * @details this function will fail if you go out of bounds on the packet size. 
 * @param wrapper_ptr the wrapper to get the body of the packet from.
 * @param offset The offset in the body of the packet to read from, so the read will start from this byte instead of 0.
 * @param dest where the read body data will be copied to 
 * @param len the expected read length of the data from the body.
 * @returns esp_err_t the operation state 
*/
esp_err_t uart_packets_get_data_from_wrapper(uart_packets_packet_wrapper_t *wrapper_ptr, size_t offset, void *dest, size_t len)
{
    // NULL check
    if(wrapper_ptr == NULL || dest == NULL){
        return ESP_ERR_NOT_FOUND;
    }
    // packet body is the are that is not the header and not the termination footer
    size_t packet_header_size = wrapper_ptr->wrapper_config.header_len;
    size_t allowed_body_len = wrapper_ptr->packet_len  - wrapper_ptr->wrapper_config.termination_len - packet_header_size;
    // see if operation goes out of packet body bounds
    if(( (offset + len) > allowed_body_len )){
        return ESP_ERR_INVALID_ARG;
    }
    char* wrapper_body_char_ptr = (char*) wrapper_ptr->packet_ptr;
    //  skip the header, skip the offset
    wrapper_body_char_ptr += packet_header_size + offset;
    // do the copy and return
    memcpy(dest,wrapper_body_char_ptr,len);
    return ESP_OK;
}

/**
 * @brief Write data from to header of the packet
 * @details this function will fail if the request goes out of header bounds defined by configuration 
 * @param wrapper_ptr The packet wrapper to write header bytes to
 * @param offset The offset in the header of the packet to start writing from, so the write will start from this byte instead of 0.
 * @param src where the write contents will be read from.
 * @param len the expected read length of the to be written data from the src pointer.
 * @returns esp_err_t the operation state
*/
esp_err_t uart_packets_set_header_data_from_wrapper(uart_packets_packet_wrapper_t *wrapper_ptr, size_t offset, void *src, size_t len)
{
    if(wrapper_ptr == NULL || src == NULL){
        return ESP_ERR_NOT_FOUND;
    }
    // see if offset goes out of bounds
    if ( ((offset + len) > wrapper_ptr->wrapper_config.header_len )){
        return ESP_ERR_INVALID_ARG;
    }    
    char* packet_char_ptr = (char*) wrapper_ptr->packet_ptr;
    // do the offset on the packet
    packet_char_ptr += offset;
    // do the copy
    memcpy(packet_char_ptr,src,len);
    return ESP_OK;
}

/**
 * @brief Write data to the body of the packet
 * @details this function will fail if you go out of bounds on the packet size. 
 * @param wrapper_ptr the wrapper to get the body of the packet from.
 * @param offset The offset in the body of the packet to start writing from, so the write will start from this byte instead of 0.
 * @param src where the write contents will be read from.
 * @param len the expected write length of the data.
 * @returns esp_err_t the operation state 
*/
esp_err_t uart_packets_set_data_from_wrapper(uart_packets_packet_wrapper_t *wrapper_ptr, size_t offset, void *src, size_t len)
{
    // NULL check
    if(wrapper_ptr == NULL || src == NULL){
        return ESP_ERR_NOT_FOUND;
    }
    // packet body is the are that is not the header and not the termination footer
    size_t packet_header_size = wrapper_ptr->wrapper_config.header_len;
    size_t allowed_body_len = wrapper_ptr->packet_len  - wrapper_ptr->wrapper_config.termination_len - packet_header_size;
    // see if operation goes out of packet body bounds
    if( ( (offset + len) > allowed_body_len )){
        return ESP_ERR_INVALID_ARG;
    }
    char* wrapper_body_char_ptr = (char*) wrapper_ptr->packet_ptr;
    //  skip the header, skip the offset
    wrapper_body_char_ptr += packet_header_size + offset;
    // do the copy and return
    memcpy(wrapper_body_char_ptr,src,len);
    return ESP_OK;
}
