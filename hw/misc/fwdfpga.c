/*
 * QEMU XDMA PCI device
 *
 * Copyright (c) 2022 Daedalean AG
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <math.h>

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/qdev-properties.h"
#include "hw/hw.h"
#include "qom/object.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qapi/error.h"

#define TYPE_PCI_FWD_FPGA_DEVICE "fwdfpga"

#define FPGA_DEVICES_NUMBER 3

typedef struct FwdFpgaState FwdFpgaState;
DECLARE_INSTANCE_CHECKER(FwdFpgaState, FWD_FPGA, TYPE_PCI_FWD_FPGA_DEVICE)

/**
 * Offset of DRAM in FPGA address space.
 */
const size_t FPGA_DRAM_OFFSET = 0x80000000;

/**
 * Length of FPGA's dedicated DRAM.
 */
const size_t FPGA_DRAM_SIZE = 1024 * 1024 * 1024;

/**
 * Offset of IQM in FPGA address space.
*/
const size_t FPGA_IQM_OFFSET = 0x44a20000; 

/**
 * Size of IQM register bank.
 */
const size_t FPGA_IQM_SIZE = 56;

/**
 * Offset of IPP in FPGA address space.
*/
const size_t FPGA_IPP_OFFSET = 0x44a10000; 

/**
 * Size of IPP register bank.
 */
const size_t FPGA_IPP_SIZE = 48;


/**
 * Constants for IQM calculations from IQM HDD.
*/
// Width in pixels of the Camera and Mapped Image.
const uint16_t IMAGE_WIDTH = 4096;
// Height in pixels of the Camera and Mapped Image.
const uint16_t IMAGE_HEIGHT = 3000;

// IQM_ALPHA is used for Mapper function.
const double IQM_ALPHA = 0.4400607;

// See Xilinx PG195 for the layout of the following structs, in particular
// tables 5, 6 for the descriptors and the "PCIe to DMA Address Map" section
// for the other structures, including tables, 40, 41, 42, 45, 48, 96, 97,
// 108-115.

#pragma pack(1)

typedef struct XdmaDescriptor {
    uint8_t control;
    uint8_t nextAdj;
    uint16_t magic;  // 0xad4b;
    uint32_t length;
    uint64_t srcAddress;
    uint64_t dstAddress;
    uint64_t nxtAddress;
} XdmaDescriptor;

typedef struct XdmaChannel {
    uint32_t identifier;  // 0x1fc0 or 0x1fc1
    uint32_t control;
    uint32_t unused1[0x0e];
    uint32_t status;
    uint32_t unused2[0x02];
    uint32_t alignment;
    uint32_t unused3[0x2c];
} XdmaChannel;

typedef struct XdmaSgdma {
    uint32_t identifier;  // 0x1fc4 or 0x1fc5
    uint32_t unused1[31];

    uint64_t descriptorAddress;
    uint32_t descriptorAdjacent;
    uint32_t descriptorCredits;
    uint32_t unused2[0x1c];
} XdmaSgdma;

typedef struct XdmaBar {
    XdmaChannel h2cChannel0;
    XdmaChannel h2cChannel1;
    uint8_t padding1[0x0e00];
    XdmaChannel c2hChannel0;
    XdmaChannel c2hChannel1;
    uint8_t padding2[0x1e00];
    uint32_t configIdentifier;
    uint8_t padding3[0x0ffc];
    XdmaSgdma h2cSgdma0;
    XdmaSgdma h2cSgdma1;
    uint8_t padding4[0x0e00];
    XdmaSgdma c2hSgdma0;
    XdmaSgdma c2hSgdma1;
    uint8_t padding5[0x2e00];
} XdmaBar;

typedef struct Iqm {
    uint32_t control;
    uint32_t status;
    uint32_t error;
    uint32_t input_baseaddr;
    uint32_t map_baseaddr;
    uint32_t bright_r;
    uint32_t bright_g;
    uint32_t bright_b;
    uint32_t bright_rgb;
    uint32_t contrast1;
    uint32_t contrast1_msb;
    uint32_t contrast2;
    uint32_t contrast2_msb;

    QemuThread thread;
    QemuMutex mutex;
    QemuCond cv;
    bool shutdown;

    void *fpga_dram;
} Iqm;

typedef struct Ipp{
    uint32_t control;
    uint32_t status;
    uint32_t error;
    uint32_t offset;
    uint32_t scale;
    uint32_t input_baseaddr;
    uint32_t input_baseaddr_msb;
    uint32_t norm_baseaddr;
    uint32_t norm_baseaddr_msb;
    uint32_t tile_baseaddr;
    uint32_t tile_baseaddr_msb;
    uint32_t reversed;

    QemuThread thread;
    QemuMutex mutex;
    QemuCond cv;
    bool shutdown;

    void *fpga_dram;
} Ipp;

typedef struct Device {
    uint64_t offset;
    uint64_t size;

    void *device;

    void (*check_status)(void *device);
} Device;

#pragma pack()

typedef enum FwdFpgaXdmaEngineDirection {
    FWD_FPGA_XDMA_ENGINE_DIRECTION_H2C,
    FWD_FPGA_XDMA_ENGINE_DIRECTION_C2H,
} FwdFpgaXdmaEngineDirection;

typedef struct FwdFpgaXdmaEngine {
    FwdFpgaXdmaEngineDirection direction;
    PCIDevice* pdev;
    XdmaChannel* channel;
    XdmaSgdma* sgdma;
    QemuMutex* bar_mutex;
    void* fpga_dram;
    Iqm* iqm;
    Ipp* ipp;

    Device* devices;

    QemuThread thread;
    QemuMutex mutex;
    QemuCond cv;
    bool running;
    bool shutdown;
} FwdFpgaXdmaEngine;

struct FwdFpgaState {
    PCIDevice pdev;
    MemoryRegion mmio;

    QemuMutex bar_mutex;
    XdmaBar bar;
    void* fpga_dram;

    Device devices[FPGA_DEVICES_NUMBER];

    uint8_t xdma_bar_id;
    uint32_t xdma_bar_size;

    FwdFpgaXdmaEngine h2c_engines[2];
    FwdFpgaXdmaEngine c2h_engines[2];

    Iqm* iqm;
    Ipp* ipp;
};

static GArray* fwdfpga_xdma_engine_fetch_descriptors(FwdFpgaXdmaEngine* engine) {
    uint64_t addr = engine->sgdma->descriptorAddress;
    uint32_t nAdj = engine->sgdma->descriptorAdjacent;
    GArray* array = g_array_new(false, false, sizeof(XdmaDescriptor));

    while (true) {
        uint32_t nPrevDescriptors = array->len;
        uint32_t nNewDescriptors = 1 + nAdj;

        g_array_set_size(array, nPrevDescriptors + nNewDescriptors);
        if (pci_dma_read(engine->pdev, addr, &g_array_index(array, XdmaDescriptor, nPrevDescriptors), nNewDescriptors * sizeof(XdmaDescriptor)) != MEMTX_OK) {
            qemu_mutex_lock(engine->bar_mutex);
            engine->channel->status |= 0x80000; // descriptor read error
            qemu_mutex_unlock(engine->bar_mutex);
            g_array_free(array, true);
            return NULL;
        }

        const XdmaDescriptor* last_descriptor = &g_array_index(array, XdmaDescriptor, array->len - 1);
        if (last_descriptor->magic != 0xad4b) {
            qemu_mutex_lock(engine->bar_mutex);
            engine->channel->status |= 0x80010; // descriptor magic error
            qemu_mutex_unlock(engine->bar_mutex);
            g_array_free(array, true);
            return NULL;
        }

        addr = last_descriptor->nxtAddress;
        nAdj = last_descriptor->nextAdj;
        if (last_descriptor->control & 0x01) { // stop bit set
            break;
        }
    }

    return array;
}

/**
 * Translate an address region in the FPGA address space to an offset into
 * the FwdFPGA's DRAM, or ~0ULL if it does not refer to the FwdFPGA's DRAM.
 */
static uint64_t translate_fpga_address_to_dram_offset(uint64_t address, uint64_t length) {
    if (address < FPGA_DRAM_OFFSET ||
        address + length > FPGA_DRAM_OFFSET + FPGA_DRAM_SIZE) {
        return ~0ULL;
    }

    return address - FPGA_DRAM_OFFSET;
}

static uint64_t translate_fpga_address_to_device_offset(Device *dev, uint64_t address, uint64_t length) {
    if (address < dev->offset ||
        address + length > dev->offset + dev->size) {
        return ~0ULL;
    }

    return address - dev->offset;
}

static void iqm_mapper(Iqm *iqm) {
    uint64_t input_offset = translate_fpga_address_to_dram_offset(iqm->input_baseaddr, 0);
    uint64_t mapped_offset = translate_fpga_address_to_dram_offset(iqm->map_baseaddr, 0);

    uint64_t pixel_offset = 0x0000000;
    for (uint16_t pixel_x = 0; pixel_x < IMAGE_WIDTH; pixel_x++) {
        for (uint16_t pixel_y = 0; pixel_y < IMAGE_HEIGHT; pixel_y++) {
            uint16_t pxl_value = 0;  
            memcpy(&pxl_value, iqm->fpga_dram + input_offset + pixel_offset, sizeof(pxl_value));

            double pxl = pxl_value;
            
            // Formula can be found in IQM HDD.
            pxl = 16 * (2.0 * sqrt(pxl) / IQM_ALPHA  - (2 * log(IQM_ALPHA * sqrt(pxl) + 1)) / (IQM_ALPHA * IQM_ALPHA));
            pxl_value = floor(pxl);

            memcpy(iqm->fpga_dram + mapped_offset + pixel_offset, &pxl_value, sizeof(pxl_value));

            pixel_offset += sizeof(pxl_value);
        }   
    }
}

static void iqm_brightness_and_contrast(Iqm *iqm) {
    uint64_t mapped_offset = translate_fpga_address_to_dram_offset(iqm->map_baseaddr, 0);
    uint64_t bayer_offsets[] = {0, 0x2, 0x2000, 0x2002}; // Addresses of RGGB pixels in Bayer pattern

    uint64_t bright_r = 0, bright_g = 0, bright_b = 0, bright_rgb = 0,
            contrast1 = 0, contrast2 = 0;

    for (uint16_t pixel_y = 0; pixel_y < IMAGE_HEIGHT/2; pixel_y++) {
        for (uint16_t pixel_x = 0; pixel_x < IMAGE_WIDTH/2; pixel_x++) {
            uint64_t baseaddress = 0x4000 * pixel_y + 0x4 * pixel_x;

            uint16_t pixel_val, mx_color = 0, gray = 0;
            uint16_t color_values[4];
            for (uint8_t pixel_idx = 0; pixel_idx < 4; pixel_idx++) {
                memcpy(&pixel_val, iqm->fpga_dram + mapped_offset + baseaddress + bayer_offsets[pixel_idx], sizeof(pixel_val));

                if (pixel_val > mx_color)
                    mx_color = pixel_val;
                gray += pixel_val;
                color_values[pixel_idx] = pixel_val;
            }
    
            bright_rgb += mx_color;

            bright_r += color_values[0];
            bright_g += color_values[1] + color_values[2];
            bright_b += color_values[3];

            contrast1 += gray;
            contrast2 += gray * gray;
        }
    }

    // Division by 8, so the results fit in a 32-bit register accroding to IQM ICD.
    iqm->bright_r = bright_r / 8;
    iqm->bright_g = bright_g / 2 / 8;
    iqm->bright_b = bright_b / 8;
    iqm->bright_rgb = bright_rgb / 8;


    contrast1 /= 4;
    iqm->contrast1 = (uint32_t)contrast1;
    iqm->contrast1_msb = contrast1 >> 32;

    contrast2 /= 16;
    iqm->contrast2 = (uint32_t)contrast2;
    iqm->contrast2_msb = contrast2 >> 32;
}

static void execute_iqm(Iqm* iqm) {
    iqm_mapper(iqm);
    iqm_brightness_and_contrast(iqm);
}

static void* fwdfpga_iqm_thread(void* context) {
    Iqm* iqm = (Iqm*)context;

    qemu_mutex_lock(&iqm->mutex);
    
    while (1) {
        qemu_cond_wait(&iqm->cv, &iqm->mutex);
        if (iqm->shutdown) {
            break;
        }

        iqm->status = 1;

        qemu_mutex_unlock(&iqm->mutex);
        execute_iqm(iqm);
        qemu_mutex_lock(&iqm->mutex);

        iqm->control = 0;
        iqm->status = 0;
    }

    qemu_mutex_unlock(&iqm->mutex);

    return NULL;
}

static void check_status_iqm(void *device) {
    Iqm* iqm = (Iqm*)device;

    qemu_mutex_lock(&iqm->mutex);
    if (iqm->control & 1 && iqm->status == 0)
        qemu_cond_signal(&iqm->cv);
    qemu_mutex_unlock(&iqm->mutex);
}

static uint8_t tile_pos(int16_t (*indices)[2], uint16_t image_size, uint16_t tile_size, uint16_t px_pos, uint16_t decimation) {
    uint8_t nr_tiles = ceil((image_size / decimation) * 1.0 / tile_size);

    // Precalculated offsets for tiling algorithm from IPP HDD.
    int16_t offsets_width[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int16_t offsets_height[] = {0, -9, -18};

	uint16_t tiled_px_pos = px_pos / decimation,
    		 index = tiled_px_pos / tile_size;

    uint8_t indices_idx = 0;
    while (index < nr_tiles) {
        int16_t offset = 0;
        if (image_size == 4096)
            offset = offsets_width[index];
        else if (image_size == 1500)
            offset = offsets_height[index];

        int16_t tile_index = (tiled_px_pos - offset) / tile_size,
        		px_index = tiled_px_pos - (index * tile_size + offset);

        index++;

        if (px_index < 0 || px_index >= tile_size)
            continue;

        indices[indices_idx][0] = tile_index;
        indices[indices_idx][1] = px_index;
        indices_idx++;
    }

	return indices_idx;
}

static uint8_t tile_xy(uint16_t (*tiles)[4], uint16_t image_width, uint16_t image_height, uint16_t tile_size, uint16_t px_x, uint16_t px_y, uint16_t decimation) {
	uint8_t tile_x_len = ceil(1.0 * image_width / decimation / tile_size),
		tile_y_len = ceil(1.0 * image_height / decimation / tile_size);

	int16_t tile_x[tile_x_len][2];
	tile_x_len = tile_pos(tile_x, image_width, tile_size, px_x, decimation);

	int16_t tile_y[tile_y_len][2];
	tile_y_len = tile_pos(tile_y, image_height, tile_size, px_y, decimation);

	uint8_t tiles_idx = 0;
	for (uint8_t i = 0; i < tile_x_len; i++)
		for (uint8_t j = 0; j < tile_y_len; j++) {
			tiles[tiles_idx][0] = tile_x[i][0];
			tiles[tiles_idx][1] = tile_y[j][0];
			tiles[tiles_idx][2] = tile_x[i][1];
			tiles[tiles_idx][3] = tile_y[j][1];

			tiles_idx++;
		}

	return tiles_idx;
}

static uint8_t tile_addresses(uint32_t *addresses, uint16_t px_x, uint16_t px_y, uint16_t image_width, uint16_t image_height, uint16_t tile_size, uint16_t decimation, uint16_t batch_size, uint16_t block_size) {
	uint16_t tiled_image_width = image_width / decimation;
    uint8_t nr_tiles = ceil(1.0 * tiled_image_width / tile_size);
    uint16_t pixel_batch_size_bytes = block_size * batch_size;
    uint32_t tile_batch_size_bytes = tile_size * tile_size * pixel_batch_size_bytes;

    uint16_t tiles[2][4];
    uint8_t tiles_sz = tile_xy(tiles, image_width, image_height, tile_size, px_x, px_y, decimation);

    uint8_t block_index = 2 * (px_y % 2) + (px_x % 2);
    uint8_t addresses_idx = 0;
    for (uint8_t idx = 0; idx < tiles_sz; idx++) {
    	uint16_t tx = tiles[idx][0],
	    		ty = tiles[idx][1],
	    		px = tiles[idx][2],
	    		py = tiles[idx][3];

    	uint16_t global_tile = tx + nr_tiles * ty,
        		 batch_tile = global_tile % batch_size,
        		 batch = global_tile / batch_size;
        uint32_t address = batch * tile_batch_size_bytes + (py * tile_size + px) * pixel_batch_size_bytes + batch_tile * block_size + block_index;
    
        addresses[addresses_idx++] = address;
    }

    return addresses_idx;
}

static void execute_ipp(Ipp* ipp) {
    double scale = (double)ipp->scale / (double)(1 << 12);

    uint64_t input_baseaddr = (((uint64_t)ipp->input_baseaddr_msb) << 32) | ((uint64_t)ipp->input_baseaddr);
    uint64_t input_offset = translate_fpga_address_to_dram_offset(input_baseaddr, 0);

    uint64_t norm_baseaddr = (((uint64_t)ipp->norm_baseaddr_msb) << 32) | ((uint64_t)ipp->norm_baseaddr);
    uint64_t norm_offset = translate_fpga_address_to_dram_offset(norm_baseaddr, 0);

    uint64_t tile_baseaddr = (((uint64_t)ipp->norm_baseaddr_msb) << 32) | ((uint64_t)ipp->tile_baseaddr);
    uint64_t tile_offset = translate_fpga_address_to_dram_offset(tile_baseaddr, 0);

    uint64_t input_pixel_offset = 0x0000000,
            norm_pixel_offset = 0x0000000;
    for (uint16_t pixel_y = 0; pixel_y < IMAGE_HEIGHT/2; pixel_y++) {
        for (uint16_t pixel_x = 0; pixel_x < IMAGE_WIDTH; pixel_x++) {
            uint16_t pxl_value = 0;  
            memcpy(&pxl_value, ipp->fpga_dram + input_offset + input_pixel_offset, sizeof(pxl_value));

            // Formula can be found in IPP HDD
            pxl_value = floor((scale * pxl_value + ipp->offset) / 16.0 + 0.5);

            uint8_t norm_pxl_value = pxl_value;            
            memcpy(ipp->fpga_dram + norm_offset + norm_pixel_offset, &norm_pxl_value, sizeof(norm_pxl_value));

            uint32_t addresses[2];
    	    uint8_t addresses_size = tile_addresses(addresses, pixel_x, pixel_y, IMAGE_WIDTH, IMAGE_HEIGHT/2, 256, 2, 12, 16);

            for (uint8_t idx = 0; idx < addresses_size; idx++)
                memcpy(ipp->fpga_dram + tile_offset + addresses[idx], &norm_pxl_value, sizeof(norm_pxl_value));

            input_pixel_offset += sizeof(pxl_value);
            norm_pixel_offset += sizeof(norm_pxl_value);
        }   
    }
}

static void* fwdfpga_ipp_thread(void* context) {
    Ipp* ipp = (Ipp*)context;

    qemu_mutex_lock(&ipp->mutex);
    
    while (1) {
        qemu_cond_wait(&ipp->cv, &ipp->mutex);

        if (ipp->shutdown) {
            break;
        }

        ipp->status = 1; 
        
        qemu_mutex_unlock(&ipp->mutex);
        execute_ipp(ipp);
        qemu_mutex_lock(&ipp->mutex);

        ipp->control = 0;
        ipp->status = 0;
    }

    qemu_mutex_unlock(&ipp->mutex);

    return NULL;
}

static void check_status_ipp(void *device) {
    Ipp* ipp = (Ipp*)device;

    qemu_mutex_lock(&ipp->mutex);
    if (ipp->control & 1 && ipp->status == 0)
        qemu_cond_signal(&ipp->cv);
    qemu_mutex_unlock(&ipp->mutex);
}

static bool fwdfpga_xdma_engine_execute_descriptor(FwdFpgaXdmaEngine* engine, const XdmaDescriptor* descriptor) {
    for (uint8_t idx = 0; idx < FPGA_DEVICES_NUMBER; idx++) {
        if (engine->direction == FWD_FPGA_XDMA_ENGINE_DIRECTION_H2C) {
            uint64_t device_offset = translate_fpga_address_to_device_offset(&engine->devices[idx], descriptor->dstAddress, descriptor->length);

            if (device_offset != ~0ULL) {
                if (pci_dma_read(engine->pdev, descriptor->srcAddress, engine->devices[idx].device + device_offset, descriptor->length) != MEMTX_OK) {
                    qemu_mutex_lock(engine->bar_mutex);
                    engine->channel->status |= 0x200; // read error
                    qemu_mutex_unlock(engine->bar_mutex);
                    return false;
                }

                if (engine->devices[idx].check_status != NULL)
                    engine->devices[idx].check_status(engine->devices[idx].device);

                return true;
            }
        } else {
            uint64_t device_offset = translate_fpga_address_to_device_offset(&engine->devices[idx], descriptor->srcAddress, descriptor->length);

            if (device_offset != ~0ULL) {
                if (pci_dma_write(engine->pdev, descriptor->dstAddress, engine->devices[idx].device + device_offset, descriptor->length) != MEMTX_OK) {
                    qemu_mutex_lock(engine->bar_mutex);
                    engine->channel->status |= 0x4000; // write error
                    qemu_mutex_unlock(engine->bar_mutex);
                    return false;
                }

                return true;
            }
        }
    }

    // If the descriptor address doesn't match any device, set an error status.
    qemu_mutex_lock(engine->bar_mutex);
    if (engine->direction == FWD_FPGA_XDMA_ENGINE_DIRECTION_H2C) 
        engine->channel->status |= 0x200; // read error
    else
        engine->channel->status |= 0x4000; // write error
    qemu_mutex_unlock(engine->bar_mutex);

    return false;
}

static void fwdfpga_xdma_engine_run(FwdFpgaXdmaEngine* engine) {
    GArray* descriptors = fwdfpga_xdma_engine_fetch_descriptors(engine);
    if (descriptors != NULL) {
        size_t i;
        for (i = 0; i < descriptors->len; ++i) {
            const XdmaDescriptor* descriptor = &g_array_index(descriptors, XdmaDescriptor, i);
            if (!fwdfpga_xdma_engine_execute_descriptor(engine, descriptor)) {
                break;
            }
        }

        if (i == descriptors->len) {
            qemu_mutex_lock(engine->bar_mutex);
            engine->channel->status |= 0x02; // stopped due to reaching last descriptor
            qemu_mutex_unlock(engine->bar_mutex);
        }

        g_array_free(descriptors, true);
    }

    qemu_mutex_lock(engine->bar_mutex);
    engine->channel->status &= ~0x01; // reset busy flag
    qemu_mutex_unlock(engine->bar_mutex);
}

static void* fwdfpga_xdma_engine_thread(void* context) {
    FwdFpgaXdmaEngine* engine = (FwdFpgaXdmaEngine*)context;

    qemu_mutex_lock(&engine->mutex);
    while (1) {
        qemu_cond_wait(&engine->cv, &engine->mutex);
        if (engine->shutdown) {
            break;
        }

        if (!engine->running) {
            continue;
        }

        qemu_mutex_unlock(&engine->mutex);
        fwdfpga_xdma_engine_run(engine);
        qemu_mutex_lock(&engine->mutex);
        engine->running = false;
    }

    qemu_mutex_unlock(&engine->mutex);
    return NULL;
}

static void fwdfpga_xdma_engine_init(FwdFpgaXdmaEngine* engine, FwdFpgaXdmaEngineDirection direction, PCIDevice* pdev, QemuMutex* bar_mutex, void *fpga_dram, Iqm *iqm, Ipp *ipp, Device *devices, XdmaChannel* channel, XdmaSgdma* sgdma) {
    engine->direction = direction;
    engine->pdev = pdev;
    engine->channel = channel;
    engine->sgdma = sgdma;
    engine->bar_mutex = bar_mutex;
    engine->fpga_dram = fpga_dram;
    engine->iqm = iqm;
    engine->ipp = ipp;
    engine->devices = devices;
    engine->running = false;
    engine->shutdown = false;

    qemu_mutex_init(&engine->mutex);
    qemu_cond_init(&engine->cv);
    qemu_thread_create(&engine->thread, "xdma", fwdfpga_xdma_engine_thread, engine, QEMU_THREAD_JOINABLE);
}

static void fwdfpga_iqm_init(Iqm *iqm, void* fpga_dram) {
    memset(iqm, 0, sizeof(*iqm));

    iqm->fpga_dram = fpga_dram;

    qemu_mutex_init(&iqm->mutex);
    qemu_cond_init(&iqm->cv);
    qemu_thread_create(&iqm->thread, "iqm", fwdfpga_iqm_thread, iqm, QEMU_THREAD_JOINABLE);
}

static void fwdfpga_iqm_uninit(Iqm* iqm) {
    qemu_mutex_lock(&iqm->mutex);
    iqm->shutdown = true;
    qemu_mutex_unlock(&iqm->mutex);
    qemu_cond_signal(&iqm->cv);

    qemu_thread_join(&iqm->thread);
    qemu_cond_destroy(&iqm->cv);
    qemu_mutex_destroy(&iqm->mutex);
}

static void fwdfpga_ipp_init(Ipp *ipp, void* fpga_dram) {
    memset(ipp, 0, sizeof(*ipp));
    
    ipp->fpga_dram = fpga_dram;

    qemu_mutex_init(&ipp->mutex);
    qemu_cond_init(&ipp->cv);
    qemu_thread_create(&ipp->thread, "ipp", fwdfpga_ipp_thread, ipp, QEMU_THREAD_JOINABLE);
}

static void fwdfpga_ipp_uninit(Ipp* ipp) {
    qemu_mutex_lock(&ipp->mutex);
    ipp->shutdown = true;
    qemu_mutex_unlock(&ipp->mutex);
    qemu_cond_signal(&ipp->cv);

    qemu_thread_join(&ipp->thread);
    qemu_cond_destroy(&ipp->cv);
    qemu_mutex_destroy(&ipp->mutex);
}

static void fwdfpga_xdma_engine_uninit(FwdFpgaXdmaEngine* engine) {
    qemu_mutex_lock(&engine->mutex);
    engine->shutdown = true;
    qemu_mutex_unlock(&engine->mutex);
    qemu_cond_signal(&engine->cv);

    qemu_thread_join(&engine->thread);
    qemu_cond_destroy(&engine->cv);
    qemu_mutex_destroy(&engine->mutex);
}

/* Start DMA engine, unless it is running already */
static bool fwdfpga_xdma_engine_start(FwdFpgaXdmaEngine* engine) {
    qemu_mutex_lock(&engine->mutex);
    if (engine->running) {
        qemu_mutex_unlock(&engine->mutex);
        return false;
    }

    // Need to set this here to avoid race condition: after control register
    // is written, status register must indicate busy immediately.
    qemu_mutex_lock(engine->bar_mutex);
    engine->channel->status = 0x01; // busy, reset error bits
    qemu_mutex_unlock(engine->bar_mutex);

    engine->running = true;
    qemu_mutex_unlock(&engine->mutex);
    qemu_cond_signal(&engine->cv);
    return true;
}

static void fwdfpga_xdma_engine_start_stop(FwdFpgaXdmaEngine* engine, const char* dir, int num, uint64_t val) {
    // Start engine if 1 is written to control register
    if (val & 1) {
        if (!fwdfpga_xdma_engine_start(engine)) {
            error_report("Failed to start %s Engine %d", dir, num);
        }
    } else {
        // TODO(armin): implement stopping
    }
}

static uint64_t fwdfpga_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    FwdFpgaState *fwdfpga = opaque;
    uint64_t val = ~0ULL;
    qemu_mutex_lock(&fwdfpga->bar_mutex);
    if (addr < sizeof(fwdfpga->bar)) {
      if (addr + size > sizeof(fwdfpga->bar)) {
        size = sizeof(fwdfpga->bar) - addr;
      }
      memcpy(&val, (uint8_t*)&fwdfpga->bar + addr, size);
    }
    qemu_mutex_unlock(&fwdfpga->bar_mutex);
    return val;
}

static void fwdfpga_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    FwdFpgaState *fwdfpga = opaque;

    switch(addr) {
    // Control registers:
    case offsetof(XdmaBar, h2cChannel0) + offsetof(XdmaChannel, control):
        fwdfpga_xdma_engine_start_stop(&fwdfpga->h2c_engines[0], "H2C", 0, val);
        break;
    case offsetof(XdmaBar, h2cChannel1) + offsetof(XdmaChannel, control):
        fwdfpga_xdma_engine_start_stop(&fwdfpga->h2c_engines[1], "H2C", 1, val);
        break;
    case offsetof(XdmaBar, c2hChannel0) + offsetof(XdmaChannel, control):
        fwdfpga_xdma_engine_start_stop(&fwdfpga->c2h_engines[0], "C2H", 0, val);
        break;
    case offsetof(XdmaBar, c2hChannel1) + offsetof(XdmaChannel, control):
        fwdfpga_xdma_engine_start_stop(&fwdfpga->c2h_engines[1], "C2H", 1, val);
        break;
    // Writable BAR regions:
    case offsetof(XdmaBar, h2cSgdma0) + offsetof(XdmaSgdma, descriptorAddress):
    case offsetof(XdmaBar, h2cSgdma0) + offsetof(XdmaSgdma, descriptorAdjacent):
    case offsetof(XdmaBar, h2cSgdma0) + offsetof(XdmaSgdma, descriptorCredits):
    case offsetof(XdmaBar, h2cSgdma1) + offsetof(XdmaSgdma, descriptorAddress):
    case offsetof(XdmaBar, h2cSgdma1) + offsetof(XdmaSgdma, descriptorAdjacent):
    case offsetof(XdmaBar, h2cSgdma1) + offsetof(XdmaSgdma, descriptorCredits):
    case offsetof(XdmaBar, c2hSgdma0) + offsetof(XdmaSgdma, descriptorAddress):
    case offsetof(XdmaBar, c2hSgdma0) + offsetof(XdmaSgdma, descriptorAdjacent):
    case offsetof(XdmaBar, c2hSgdma0) + offsetof(XdmaSgdma, descriptorCredits):
    case offsetof(XdmaBar, c2hSgdma1) + offsetof(XdmaSgdma, descriptorAddress):
    case offsetof(XdmaBar, c2hSgdma1) + offsetof(XdmaSgdma, descriptorAdjacent):
    case offsetof(XdmaBar, c2hSgdma1) + offsetof(XdmaSgdma, descriptorCredits):
        qemu_mutex_lock(&fwdfpga->bar_mutex);
        memcpy((uint8_t*)&fwdfpga->bar + addr, &val, size);
        qemu_mutex_unlock(&fwdfpga->bar_mutex);
        break;
    default:
        error_report("attempt to write to non-writable address 0x%lx", (unsigned long)addr);
        break;
    }
}

static const MemoryRegionOps fwdfpga_mmio_ops = {
    .read = fwdfpga_mmio_read,
    .write = fwdfpga_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 8,
    },
};

static void pci_fwdfpga_realize(PCIDevice *pdev, Error **errp)
{
    FwdFpgaState *fwdfpga = FWD_FPGA(pdev);

    int dram_fd = open("/tmp", O_TMPFILE | O_RDWR | O_EXCL, 0600);
    if (dram_fd == -1) {
        error_setg_errno(errp, errno, "Failed to open FPGA DRAM tmp file");
        return;
    }

    if (ftruncate(dram_fd, FPGA_DRAM_SIZE) == -1) {
        error_setg_errno(errp, errno, "Failed to truncate FPGA DRAM tmp file");
        close(dram_fd);
        return;
    }

    fwdfpga->fpga_dram = mmap(NULL, FPGA_DRAM_SIZE, PROT_READ | PROT_WRITE, MAP_PRIVATE, dram_fd, 0);
    if (fwdfpga->fpga_dram == MAP_FAILED) {
        error_setg_errno(errp, errno, "Failed to map FPGA DRAM tmp file");
        close(dram_fd);
        return;
    }

    if (close(dram_fd) != 0) {
        error_setg_errno(errp, errno, "Failed to close FPGA DRAM tmp file");
        return;
    }

    qemu_mutex_init(&fwdfpga->bar_mutex);

    const XdmaBar bar = {
            .h2cChannel0 = {.identifier = 0x1fc00006, .alignment = 0x00010106},
            .h2cChannel1 = {.identifier = 0x1fc00106, .alignment = 0x00010106},
            .c2hChannel0 = {.identifier = 0x1fc10006, .alignment = 0x00010106},
            .c2hChannel1 = {.identifier = 0x1fc10106, .alignment = 0x00010106},
            .configIdentifier = 0x1fc30000,
            .h2cSgdma0 = {.identifier = 0x1fc40006},
            .h2cSgdma1 = {.identifier = 0x1fc40106},
            .c2hSgdma0 = {.identifier = 0x1fc50006},
            .c2hSgdma1 = {.identifier = 0x1fc50106},
    };

    fwdfpga->bar = bar;

    fwdfpga->iqm = g_malloc(sizeof(Iqm));
    fwdfpga_iqm_init(fwdfpga->iqm, fwdfpga->fpga_dram);
    
    fwdfpga->ipp = g_malloc(sizeof(Ipp));
    fwdfpga_ipp_init(fwdfpga->ipp, fwdfpga->fpga_dram);

    fwdfpga->devices[0] = (struct Device) {
        .offset = FPGA_DRAM_OFFSET,
        .size = FPGA_DRAM_SIZE,
        .device = fwdfpga->fpga_dram,
        .check_status = NULL
    };

    fwdfpga->devices[1] = (struct Device) {
        .offset = FPGA_IQM_OFFSET,
        .size = FPGA_IQM_SIZE,
        .device = fwdfpga->iqm,
        .check_status = check_status_iqm
    };
    
    fwdfpga->devices[2] = (struct Device) {
        .offset = FPGA_IPP_OFFSET,
        .size = FPGA_IPP_SIZE,
        .device = fwdfpga->ipp,
        .check_status = check_status_ipp
    };

    fwdfpga_xdma_engine_init(&fwdfpga->h2c_engines[0], FWD_FPGA_XDMA_ENGINE_DIRECTION_H2C, &fwdfpga->pdev, &fwdfpga->bar_mutex, fwdfpga->fpga_dram, fwdfpga->iqm, fwdfpga->ipp, fwdfpga->devices, &fwdfpga->bar.h2cChannel0, &fwdfpga->bar.h2cSgdma0);
    fwdfpga_xdma_engine_init(&fwdfpga->h2c_engines[1], FWD_FPGA_XDMA_ENGINE_DIRECTION_H2C, &fwdfpga->pdev, &fwdfpga->bar_mutex, fwdfpga->fpga_dram, fwdfpga->iqm, fwdfpga->ipp, fwdfpga->devices, &fwdfpga->bar.h2cChannel1, &fwdfpga->bar.h2cSgdma1);
    fwdfpga_xdma_engine_init(&fwdfpga->c2h_engines[0], FWD_FPGA_XDMA_ENGINE_DIRECTION_C2H, &fwdfpga->pdev, &fwdfpga->bar_mutex, fwdfpga->fpga_dram, fwdfpga->iqm, fwdfpga->ipp, fwdfpga->devices, &fwdfpga->bar.c2hChannel0, &fwdfpga->bar.c2hSgdma0);
    fwdfpga_xdma_engine_init(&fwdfpga->c2h_engines[1], FWD_FPGA_XDMA_ENGINE_DIRECTION_C2H, &fwdfpga->pdev, &fwdfpga->bar_mutex, fwdfpga->fpga_dram, fwdfpga->iqm, fwdfpga->ipp, fwdfpga->devices, &fwdfpga->bar.c2hChannel1, &fwdfpga->bar.c2hSgdma1);

    memory_region_init_io(&fwdfpga->mmio, OBJECT(fwdfpga), &fwdfpga_mmio_ops, fwdfpga,
            "fwdfpga-mmio", fwdfpga->xdma_bar_size);
    pci_register_bar(pdev, fwdfpga->xdma_bar_id, PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_64, &fwdfpga->mmio);
}

static void pci_fwdfpga_uninit(PCIDevice *pdev)
{
    FwdFpgaState *fwdfpga = FWD_FPGA(pdev);

    fwdfpga_xdma_engine_uninit(&fwdfpga->h2c_engines[0]);
    fwdfpga_xdma_engine_uninit(&fwdfpga->h2c_engines[1]);
    fwdfpga_xdma_engine_uninit(&fwdfpga->c2h_engines[0]);
    fwdfpga_xdma_engine_uninit(&fwdfpga->c2h_engines[1]);

    fwdfpga_iqm_uninit(fwdfpga->iqm);
    fwdfpga_ipp_uninit(fwdfpga->ipp);
    g_free(fwdfpga->iqm);
    g_free(fwdfpga->ipp);

    qemu_mutex_destroy(&fwdfpga->bar_mutex);

    if (fwdfpga->fpga_dram != NULL) {
        if (munmap(fwdfpga->fpga_dram, FPGA_DRAM_SIZE)) {
            error_report("Failed to unmap file: %s", strerror(errno));
        }
    }
}

static Property fwdfga_props[] = {
        DEFINE_PROP_UINT8("xdma-bar-id", FwdFpgaState, xdma_bar_id, 0),
        DEFINE_PROP_UINT32("xdma-bar-size", FwdFpgaState, xdma_bar_size, sizeof(XdmaBar)),
        DEFINE_PROP_END_OF_LIST()
};

static void fwdfpga_instance_init(Object *obj)
{
    FwdFpgaState *fwdfpga = FWD_FPGA(obj);
    fwdfpga->fpga_dram = NULL;
}

static void fwdfpga_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(class);

    k->realize = pci_fwdfpga_realize;
    k->exit = pci_fwdfpga_uninit;
    k->vendor_id = 0x10ee; // Xilinx
    k->device_id = 0xdd01;
    k->revision = 0x10;
    k->class_id = PCI_CLASS_OTHERS;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);

    device_class_set_props(dc, fwdfga_props);
}

static void pci_fwdfpga_register_types(void)
{
    static InterfaceInfo interfaces[] = {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    };
    static const TypeInfo fwdfpga_info = {
        .name          = TYPE_PCI_FWD_FPGA_DEVICE,
        .parent        = TYPE_PCI_DEVICE,
        .instance_size = sizeof(FwdFpgaState),
        .instance_init = fwdfpga_instance_init,
        .class_init    = fwdfpga_class_init,
        .interfaces = interfaces,
    };

    type_register_static(&fwdfpga_info);
}
type_init(pci_fwdfpga_register_types)
