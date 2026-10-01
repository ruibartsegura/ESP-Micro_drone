/*
 * test_bmp180.c
 *
 * Tests Unity (framework incluido en ESP-IDF) para bmp180.c.
 *
 * Solo se testea la MATEMATICA PURA de compensacion (bmp180_compute_b5,
 * bmp180_compensate_temperature, bmp180_compensate_pressure,
 * bmp180_pressure_to_altitude) y los guardas de NULL / entrada invalida
 * en la API publica. NO se testea bmp180_init(), bmp180_read_temperature()
 * ni bmp180_read_pressure() "de verdad" (mas alla de sus checks de NULL)
 * porque requieren hardware I2C real -- eso se comprueba a mano en el
 * dron, no aqui.
 *
 * Como ejecutar (dos opciones):
 *
 *  A) Como parte de la app normal, en una tarea de depuracion:
 *       #include "unity.h"
 *       ...
 *       UNITY_BEGIN();
 *       unity_run_all_tests();
 *       UNITY_END();
 *
 *  B) Con la app de test de ESP-IDF (recomendado, mas aislado):
 *       idf.py -T height flash monitor
 *     usando el "unit-test-app" de $IDF_PATH/tools/unit-test-app como base
 *     de proyecto y REQUIRES el componente 'height' desde su sdkconfig.
 */

#include <math.h>
#include "unity.h"
#include "bmp180.h"
#include "bmp180_internal.h"

// ---------------------------------------------------------------------
// Caso de referencia oficial: Figura 4 del datasheet Bosch BMP180
// (BST-BMP180-DS000-07, pagina 15). Estos son los valores de ejemplo
// que trae el propio fabricante para verificar una implementacion.
// ---------------------------------------------------------------------
static bmp180_t make_datasheet_example_dev(void)
{
    bmp180_t dev = {0};
    dev.mode = BMP180_MODE_ULTRA_LOW_POWER; // oss = 0, como usa el ejemplo
    dev.AC1 = 408;
    dev.AC2 = -72;
    dev.AC3 = -14383;
    dev.AC4 = 32741;
    dev.AC5 = 32757;
    dev.AC6 = 23153;
    dev.B1  = 6190;
    dev.B2  = 4;
    dev.MB  = -32767;
    dev.MC  = -8711;
    dev.MD  = 2868;
    return dev;
}

#define DATASHEET_UT 27898
#define DATASHEET_UP 23843

// ---------------------------------------------------------------------
// 1. bmp180_compute_b5
// ---------------------------------------------------------------------

TEST_CASE("bmp180_compute_b5 coincide con el ejemplo del datasheet (B5=2399)", "[bmp180]")
{
    bmp180_t dev = make_datasheet_example_dev();

    int32_t b5 = bmp180_compute_b5(&dev, DATASHEET_UT);

    TEST_ASSERT_EQUAL_INT32(2399, b5);
}

// ---------------------------------------------------------------------
// 2. bmp180_compensate_temperature
// ---------------------------------------------------------------------

TEST_CASE("bmp180_compensate_temperature coincide con el ejemplo del datasheet (15.0 C)", "[bmp180]")
{
    float temperature_c = -999.0f;

    // B5 = 2399 (viene del test anterior / del ejemplo del datasheet)
    bmp180_compensate_temperature(2399, &temperature_c);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 15.0f, temperature_c);
}

// ---------------------------------------------------------------------
// 3. bmp180_compensate_pressure
// ---------------------------------------------------------------------

TEST_CASE("bmp180_compensate_pressure coincide con el ejemplo del datasheet (69965 Pa)", "[bmp180]")
{
    bmp180_t dev = make_datasheet_example_dev();
    int32_t pressure_pa = -1;

    esp_err_t err = bmp180_compensate_pressure(&dev, 2399, DATASHEET_UP, &pressure_pa);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_INT32(69965, pressure_pa);
}

TEST_CASE("bmp180_compensate_pressure con dev NULL devuelve ESP_ERR_INVALID_ARG", "[bmp180]")
{
    int32_t pressure_pa;
    esp_err_t err = bmp180_compensate_pressure(NULL, 2399, DATASHEET_UP, &pressure_pa);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

TEST_CASE("bmp180_compensate_pressure con pressure_pa NULL devuelve ESP_ERR_INVALID_ARG", "[bmp180]")
{
    bmp180_t dev = make_datasheet_example_dev();
    esp_err_t err = bmp180_compensate_pressure(&dev, 2399, DATASHEET_UP, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

TEST_CASE("bmp180_compensate_pressure con AC4=0 no crashea (division por cero evitada)", "[bmp180]")
{
    // AC4=0 fuerza b4=0 en la formula interna. Sin el guard anadido,
    // esto seria una division entre cero en tiempo de ejecucion.
    bmp180_t dev = make_datasheet_example_dev();
    dev.AC4 = 0;
    int32_t pressure_pa = -1;

    esp_err_t err = bmp180_compensate_pressure(&dev, 2399, DATASHEET_UP, &pressure_pa);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

// Todos los modos de oversampling deben devolver ESP_OK con datos validos,
// y la presion no deberia dispararse a un valor absurdo por el cambio de oss.
TEST_CASE("bmp180_compensate_pressure funciona en los 4 modos de oversampling", "[bmp180]")
{
    bmp180_t dev = make_datasheet_example_dev();

    for (int mode = BMP180_MODE_ULTRA_LOW_POWER; mode <= BMP180_MODE_ULTRA_HIGH_RES; mode++) {
        dev.mode = (bmp180_mode_t)mode;
        int32_t pressure_pa = 0;

        esp_err_t err = bmp180_compensate_pressure(&dev, 2399, DATASHEET_UP, &pressure_pa);

        TEST_ASSERT_EQUAL(ESP_OK, err);
        // Rango fisico razonable: 300-1100 hPa segun el propio datasheet
        // (30000-110000 Pa). Si esto falla, algo en la formula overflowea
        // para ese modo.
        TEST_ASSERT_GREATER_THAN_INT32(30000, pressure_pa);
        TEST_ASSERT_LESS_THAN_INT32(110000, pressure_pa);
    }
}

// ---------------------------------------------------------------------
// 4. bmp180_pressure_to_altitude
// ---------------------------------------------------------------------

TEST_CASE("bmp180_pressure_to_altitude da 0 m cuando presion == presion de referencia", "[bmp180]")
{
    float altitude = bmp180_pressure_to_altitude(101325, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, altitude);
}

TEST_CASE("bmp180_pressure_to_altitude usa 101325 Pa por defecto si sea_level_pa <= 0", "[bmp180]")
{
    float altitude_explicit = bmp180_pressure_to_altitude(100000, 101325.0f);
    float altitude_default_zero = bmp180_pressure_to_altitude(100000, 0.0f);
    float altitude_default_negative = bmp180_pressure_to_altitude(100000, -5.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, altitude_explicit, altitude_default_zero);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, altitude_explicit, altitude_default_negative);
}

TEST_CASE("bmp180_pressure_to_altitude valor de referencia (100000 Pa -> ~110.9 m)", "[bmp180]")
{
    // Calculado con la formula barometrica en Python para tener un
    // valor de referencia independiente del propio codigo bajo test.
    float altitude = bmp180_pressure_to_altitude(100000, 101325.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 110.90f, altitude);
}

TEST_CASE("bmp180_pressure_to_altitude con presion <= 0 devuelve NaN en vez de crashear", "[bmp180]")
{
    TEST_ASSERT_TRUE(isnan(bmp180_pressure_to_altitude(0, 101325.0f)));
    TEST_ASSERT_TRUE(isnan(bmp180_pressure_to_altitude(-100, 101325.0f)));
}

// ---------------------------------------------------------------------
// 5. Guardas de NULL en la API publica (no tocan I2C, se puede probar
//    sin sensor conectado porque el check de NULL es lo primero que
//    hace cada funcion)
// ---------------------------------------------------------------------

TEST_CASE("bmp180_read_temperature con dev NULL devuelve ESP_ERR_INVALID_ARG", "[bmp180]")
{
    float temperature_c;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bmp180_read_temperature(NULL, &temperature_c));
}

TEST_CASE("bmp180_read_temperature con temperature_c NULL devuelve ESP_ERR_INVALID_ARG", "[bmp180]")
{
    bmp180_t dev = make_datasheet_example_dev();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bmp180_read_temperature(&dev, NULL));
}

TEST_CASE("bmp180_read_pressure con dev NULL devuelve ESP_ERR_INVALID_ARG", "[bmp180]")
{
    int32_t pressure_pa;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bmp180_read_pressure(NULL, &pressure_pa));
}

TEST_CASE("bmp180_read_pressure con pressure_pa NULL devuelve ESP_ERR_INVALID_ARG", "[bmp180]")
{
    bmp180_t dev = make_datasheet_example_dev();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bmp180_read_pressure(&dev, NULL));
}
