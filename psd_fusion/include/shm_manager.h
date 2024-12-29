
#ifndef APA_SHM_MANAGER
#define APA_SHM_MANAGER

#include <string>
#include <mutex>
#include <condition_variable>

#include "sys/ipc.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/mman.h"
#include "fcntl.h"
#include "errno.h"
#include "apa_log.h"
#include "apa_define.h"
#include <unistd.h>

class shm_manager
{
private:
    /* data */
    int m_shm_id;
    
    int m_one_cache_size;
    int m_shm_size;
    std::string m_shm_path;

    uint8_t* m_shm_buf;
    
    int m_shm_buf_idx;

    std::mutex m_mtx;
    int is_released[IMAGE_CACHE_SIZE];
    int m_start_clock[IMAGE_CACHE_SIZE];

    std::mutex m_cache_mtx[IMAGE_CACHE_SIZE];
    std::condition_variable m_cv[IMAGE_CACHE_SIZE];

public:
    shm_manager(std::string path, int one_cache_size);
    ~shm_manager();

    int get_usable_buf_idx();
    void release_buf_idx(int shm_buf_idx);
    void* get_buffer_addr(int buf_idx, int offset_in_one_cache = 0);
    void write_to_buf(void* src, int src_size, int shm_buf_idx, int offset_in_one_cache = 0);
    bool InitSharedMemory_for_read();
    bool InitSharedMemory_for_write();
    void UnInitSharedMemory();
};

#endif