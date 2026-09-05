#include "stored_map.h"
#include "stored_table.h"
#include "adaptation/shift_adaptation.h"
#include "nvs/eeprom_config.h"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <string>
#include <vector>

// Narrow host adapters for the lookup storage layer. Production StoredMap,
// StoredTable, StoredData and ShiftAdaptationSystem are compiled unchanged.
static int allocations = 0;
LookupAllocHeader::LookupAllocHeader(const int16_t* src, uint16_t count) {
    size = count;
    header = new int16_t[count];
    std::copy_n(src, count, header);
    allocation_successful = true;
    ++allocations;
}
LookupAllocHeader::~LookupAllocHeader() { if (allocation_successful) { delete[] header; --allocations; } }
LookupAllocTable::LookupAllocTable(const int16_t* x, uint16_t xs, const int16_t* src, uint16_t count) {
    xHeader = new LookupAllocHeader(x, xs);
    xHeaderSize = xs;
    dataSize = count;
    data = new int16_t[count];
    std::copy_n(src, count, data);
    allocation_successful = true;
    ++allocations;
}
LookupAllocTable::~LookupAllocTable() { delete[] data; delete xHeader; --allocations; }
bool LookupAllocTable::is_allocated() const { return allocation_successful; }
bool LookupAllocTable::add_data(const int16_t* src, uint16_t count) {
    if (count != dataSize) return false;
    std::copy_n(src, count, data);
    return true;
}
int16_t* LookupTable::get_current_data() { return data; }
LookupAllocMap::LookupAllocMap(const int16_t* x, uint16_t xs, const int16_t* y, uint16_t ys, const int16_t* src, uint16_t count) {
    table = new LookupAllocTable(x, xs, src, count);
    yHeader = new LookupAllocHeader(y, ys);
    yHeaderSize = ys;
}
LookupAllocMap::~LookupAllocMap() { delete table; delete yHeader; }
bool LookupAllocMap::is_allocated() const { return true; }
bool LookupAllocMap::add_data(const int16_t* src, uint16_t count) {
    return static_cast<LookupAllocTable*>(table)->add_data(src, count);
}
int16_t* LookupMap::get_current_data() const { return table->get_current_data(); }
uint16_t LookupMap::data_size() { return table->data_size(); }
uint16_t LookupTable::data_size() const { return dataSize; }

static esp_err_t read_result = ESP_OK;
static const char* fail_key = nullptr;
static unsigned reads = 0;
static std::vector<std::string> saved_keys;
static const int16_t stored_values[8] = {11, 22, 33, 44, 55, 66, 77, 88};

esp_err_t EEPROM::read_nvs_map_data(const char* key, int16_t* dest, const int16_t*, size_t count) {
    ++reads;
    assert(key != nullptr && count > 0 && count <= 8);
    if (read_result != ESP_OK || (fail_key != nullptr && std::strcmp(key, fail_key) == 0)) {
        dest[0] = -999; // A failed/partial read must never become live calibration.
        return read_result == ESP_OK ? ESP_FAIL : read_result;
    }
    std::copy_n(stored_values, count, dest);
    return ESP_OK;
}
esp_err_t EEPROM::write_nvs_map_data(const char* key, const int16_t*, size_t count) {
    assert(key != nullptr && count > 0 && count <= 8);
    saved_keys.emplace_back(key);
    return ESP_OK;
}

void test_persistent_maps() {
    const int16_t x[2] = {0, 1}, y[1] = {0}, defaults[2] = {1, 2};
    const int16_t live[2] = {101, 202};
    {
        StoredMap map("map", 2, x, y, 2, 1, defaults);
        StoredTable table("table", 2, x, 2, defaults);
        assert(map.init_status() == ESP_OK && table.init_status() == ESP_OK);
        assert(map.replace_data_content(live, 2) == ESP_OK);
        assert(table.replace_data_content(live, 2) == ESP_OK);
        for (esp_err_t failure : {ESP_FAIL, ESP_ERR_INVALID_SIZE}) {
            read_result = failure;
            assert(map.reload_from_eeprom() == failure);
            assert(table.reload_from_eeprom() == failure);
            assert(std::equal(live, live + 2, map.get_current_data()));
            assert(std::equal(live, live + 2, table.get_current_data()));
        }
        read_result = ESP_OK;
        const unsigned prior_reads = reads;
        StoredData& map_interface = map;
        assert(map_interface.read_from_eeprom("map", 1) == ESP_ERR_INVALID_SIZE);
        assert(table.read_from_eeprom("table", 1) == ESP_ERR_INVALID_SIZE);
        assert(reads == prior_reads);
        assert(map.reload_from_eeprom() == ESP_OK);
        assert(table.reload_from_eeprom() == ESP_OK);
        assert(std::equal(stored_values, stored_values + 2, map.get_current_data()));
        assert(std::equal(stored_values, stored_values + 2, table.get_current_data()));
    }
    assert(allocations == 0); // Virtual base deletion must release nested storage.
    {
        read_result = ESP_FAIL;
        StoredMap failed("failed", 2, x, y, 2, 1, defaults);
        assert(failed.init_status() == ESP_FAIL);
        assert(std::strcmp(failed.get_data_name(), "failed") == 0);
        const size_t before = saved_keys.size();
        assert(failed.save_to_eeprom() == ESP_FAIL);
        assert(failed.reset_from_flash() == ESP_FAIL);
        assert(failed.reload_from_eeprom() == ESP_FAIL);
        assert(saved_keys.size() == before);
    }
    read_result = ESP_OK;
    fail_key = "apply";
    {
        ShiftAdaptationSystem adaptation;
        assert(adaptation.applying_torque_offset == nullptr);
        assert(adaptation.get_applying_torque_offset(0) == 0);
        adaptation.offset_applying_trq(0, 100); // No write through rejected object.
        assert(adaptation.get_prefill_cycles_offset(0) == stored_values[0]);
        saved_keys.clear();
        assert(adaptation.save() == ESP_OK);
        assert(saved_keys == std::vector<std::string>({"prefill", "free", "spc"}));
    }
    fail_key = nullptr;
    assert(allocations == 0);
}
