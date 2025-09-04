# Double Buffering Implementation for BLE Data Transmission

## Overview
This implementation provides a sophisticated double buffering and ring buffer system that allows continuous ADC data acquisition while simultaneously transmitting data via BLE notifications. This solves the problem of data loss during BLE transmission delays.

## Key Features

### 1. Double Buffering System
- **Two Data Buffers**: `data_buffers[0]` and `data_buffers[1]` (244 bytes each)
- **Separate Acquisition and Transmission**: One buffer fills with new data while the other transmits
- **Automatic Buffer Swapping**: Buffers swap roles after successful transmission
- **Thread-Safe Operations**: Uses atomic flags to prevent corruption during buffer switches

### 2. Ring Buffer for High-Speed Acquisition
- **512-Sample Ring Buffer**: Stores raw ADC values for ultra-fast data collection
- **Lock-Free Operations**: Uses power-of-2 sizing for efficient indexing
- **Automatic Mode Selection**: Engages when sampling rate ≥ 1000 Hz
- **Continuous Data Collection**: ADC sampling runs independently of BLE transmission

### 3. Three-Tier Work Queue System

#### Acquisition Work (`acquisition_work_handler`)
- Fills data packets from either direct ADC or ring buffer
- Manages double buffer switching
- Schedules transmission work

#### Transmission Work (`transmission_work_handler`)
- Sends BLE notifications asynchronously
- Handles BLE buffer full conditions with retry logic
- Manages buffer swapping after successful transmission

#### High-Speed ADC Work (`high_speed_adc_work_handler`)
- Continuous ADC sampling for ring buffer
- Runs at 10x transmission rate for high sampling frequencies
- Prevents data loss during BLE transmission delays

## Performance Characteristics

### Standard Mode (< 1000 Hz)
- Uses double buffering only
- Minimum 20ms acquisition interval
- Suitable for most sensor applications

### High-Speed Mode (≥ 1000 Hz)
- Uses ring buffer + double buffering
- Continuous ADC acquisition at 1ms intervals
- Can maintain high sample rates even with BLE delays

## Buffer Management

### Buffer States
- `current_acquisition_buffer`: Buffer being filled with new data
- `current_transmission_buffer`: Buffer being sent via BLE
- `buffer_ready_for_transmission`: Flag indicating data ready to send
- `acquisition_in_progress`: Prevents buffer switching during ADC operations

### Ring Buffer Operations
```c
// Thread-safe ring buffer operations
bool ring_buffer_put(uint16_t data);     // Add sample
bool ring_buffer_get(uint16_t *data);    // Get sample
uint32_t ring_buffer_available_data();   // Check available samples
```

## Error Handling

### BLE Buffer Full
- Retries transmission after 10ms delay
- Doesn't stop data acquisition
- Logs warnings for monitoring

### Ring Buffer Overflow
- Drops oldest samples when buffer is full
- Logs warnings for debugging
- Continues operation without stopping

## Configuration Options

### Automatic Mode Selection
```c
if (samplingRate >= 1000) {
    use_ring_buffer = true;  // High-speed mode
} else {
    use_ring_buffer = false; // Standard double buffering
}
```

### Timing Parameters
- **Minimum acquisition interval**: 20ms (standard mode)
- **High-speed ADC interval**: 1ms (high-speed mode)
- **BLE retry delay**: 10ms (on buffer full)

## API Changes

### Start/Stop Functions
- `start_continuous_measurement()`: Automatically selects optimal mode
- `stop_continuous_measurement()`: Stops all work items and resets buffers

### Data Packet Preparation
- `prepare_data_packet_buffered()`: Enhanced version with ring buffer support
- `prepare_data_packet_from_ring_buffer()`: Uses pre-collected ring buffer data

## Benefits

1. **No Data Loss**: Continuous acquisition even during BLE transmission
2. **Higher Throughput**: Can handle sampling rates up to 4000 Hz
3. **Reduced Latency**: Overlapped acquisition and transmission
4. **Robust Error Handling**: Graceful degradation on BLE issues
5. **Scalable Architecture**: Automatically adapts to sampling requirements

## Memory Usage

- **Double Buffers**: 2 × 244 bytes = 488 bytes
- **Ring Buffer**: 512 × 2 bytes = 1024 bytes
- **Total Additional**: ~1.5 KB RAM

## Hardware Requirements

### Single CPU Implementation
- This implementation runs efficiently on a single CPU core
- Uses cooperative scheduling (Zephyr work queues)
- No need for dual-core hardware

### When to Consider Dual CPU
Consider dual CPU only if you need:
- Sample rates > 10 kHz consistently
- Multiple concurrent sensor streams
- Real-time processing with guaranteed timing
- Dedicated DSP operations

## Usage Example

```c
// Configure sampling rate
samplingRate = 2000; // 2 kHz

// Start continuous measurement
start_continuous_measurement();

// System automatically:
// 1. Detects high sampling rate
// 2. Enables ring buffer mode
// 3. Starts high-speed ADC acquisition
// 4. Manages double buffering for BLE transmission

// Stop measurement
stop_continuous_measurement();
```

## Monitoring and Debugging

### Log Messages
- Buffer swap operations: `LOG_DBG("Buffers swapped: acq=%d, trans=%d")`
- Ring buffer status: `LOG_DBG("Filled %d samples (%d from ring buffer)")`
- BLE transmission: `LOG_DBG("Sent data packet from buffer %d")`

### Performance Metrics
- Monitor ring buffer utilization
- Track BLE transmission success rate
- Observe buffer swap frequency

This implementation provides a robust, scalable solution for high-speed data acquisition with BLE transmission, eliminating the need for dual-CPU solutions in most applications.
