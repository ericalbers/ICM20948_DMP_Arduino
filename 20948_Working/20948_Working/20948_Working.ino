
#include <Icm20948.h>
#include <SensorTypes.h>
#include "Icm20948MPUFifoControl.h"

#include <Wire.h>
#define AK0991x_DEFAULT_I2C_ADDR  0x0C  /* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E  /* The secondary I2C address for AK0991x Magnetometers */

#define ICM_I2C_ADDR_REVA      0x68  /* I2C slave address for INV device on Rev A board */
#define ICM_I2C_ADDR_REVB     0x69  /* I2C slave address for INV device on Rev B board */

#define AD0_VAL   1     // The value of the last bit of the I2C address.

#define MPU0_AD0_PIN 25 //14
#define MPU1_AD0_PIN 26 //27
#define MPU2_AD0_PIN 27 //26
#define MPU3_AD0_PIN 14 //25

int CURRENT_IMU = 0;
uint8_t I2C_Address = 0x69;
const uint8_t AD0_PINS[4] =
{
  MPU0_AD0_PIN,
  MPU1_AD0_PIN,
  MPU2_AD0_PIN,
  MPU3_AD0_PIN
};

char eamessage[1024];

static const uint8_t dmp3_image[] = 
{
#include "icm20948_img.dmp3a.h"
};


inv_icm20948_t icm_device;

int rc = 0;
#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];

/* FSR configurations */
int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

/*
* Mounting matrix configuration applied for Accel, Gyro and Mag
*/

static const float cfg_mounting_matrix[9] = {
  1.f, 0, 0,
  0, 1.f, 0,
  0, 0, 1.f
};


static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
  INV_SENSOR_TYPE_ACCELEROMETER,
  INV_SENSOR_TYPE_GYROSCOPE,
  INV_SENSOR_TYPE_RAW_ACCELEROMETER,
  INV_SENSOR_TYPE_RAW_GYROSCOPE,
  INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
  INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
  INV_SENSOR_TYPE_BAC,
  INV_SENSOR_TYPE_STEP_DETECTOR,
  INV_SENSOR_TYPE_STEP_COUNTER,
  INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
  INV_SENSOR_TYPE_ROTATION_VECTOR,
  INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
  INV_SENSOR_TYPE_MAGNETOMETER,
  INV_SENSOR_TYPE_SMD,
  INV_SENSOR_TYPE_PICK_UP_GESTURE,
  INV_SENSOR_TYPE_TILT_DETECTOR,
  INV_SENSOR_TYPE_GRAVITY,
  INV_SENSOR_TYPE_LINEAR_ACCELERATION,
  INV_SENSOR_TYPE_ORIENTATION,
  INV_SENSOR_TYPE_B2S
};

/*
* Sleep implementation for ICM20948
*/
void inv_icm20948_sleep(int ms)
{
  delay(ms);
}

void inv_icm20948_sleep_us(int us)
{
  delayMicroseconds(us);
}


void selectMPU(int m)
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(AD0_PINS[i], HIGH);
  }

  delay(25);
  if (m < 0 || m >= 4)
  {
    return;
  }
  digitalWrite(AD0_PINS[m], LOW);
  delay(25);
  Serial.print("SELECTED AD0 on DEVICE ");
  Serial.print(m);
  Serial.print(" Pin ");
  Serial.println(AD0_PINS[m]);
}


int i2c_master_write_register(uint8_t address, uint8_t reg, uint32_t len, const uint8_t *data)
{
  if (address != 0x69)
  {

    Serial.print("Odd address:");
    Serial.println(address);
  }
  //Serial.print("write address ");
  //Serial.println(address);
  //Serial.print("register ");
  //Serial.println(reg);
  //Serial.print("length = ");
  //Serial.println(len);
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data, len);
  Wire.endTransmission();
  return 0;
}

int i2c_master_read_register(uint8_t address, uint8_t reg, uint32_t len, uint8_t *buff)
{
  if (address != 0x69)
  {

    Serial.print("Odd read address:");
    Serial.println(address);
  }
  //Serial.print("read address ");
  //Serial.println(address);
  //Serial.print("register ");
  //Serial.println(reg);
  //Serial.print("length = ");
  //Serial.println(len);

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // Send repeated start

  uint32_t offset = 0;
  uint32_t num_received = Wire.requestFrom(address, len);
  //Serial.print("received = ");
  //Serial.println(num_received);
  //Serial.println(buff[0]);
  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      buff[i] = Wire.read();
    }
    return 0;
  }
  else
  {
    return -1;
  }
}



//---------------------------------------------------------------------
int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
  return i2c_master_read_register(I2C_Address, reg, rlen, rbuffer);
}

//---------------------------------------------------------------------

int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
  return i2c_master_write_register(I2C_Address, reg, wlen, wbuffer);
}

//---------------------------------------------------------------------
inv_bool_t interface_is_SPI(void)
{
  return false;
}

//---------------------------------------------------------------------
static void icm20948_apply_mounting_matrix(void)
{
  int ii;
  for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++)
  {
    inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (inv_icm20948_sensor)ii);
  }
}

//---------------------------------------------------------------------

static void icm20948_set_fsr(void)
{
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}


//--------------------------------------------------------------------


int icm20948_sensor_setup(void)
{
  int rc;
  uint8_t i, whoami = 0xff;

  /*
  * Just get the whoami
  */
  rc = inv_icm20948_get_whoami(&icm_device, &whoami);
  Serial.print("whoami = ");
  Serial.println(whoami);

  //delay(1000);

  /* Setup accel and gyro mounting matrix and associated angle for current board */
  inv_icm20948_init_matrix(&icm_device);

  Serial.print("dmp image size = ");
  Serial.println(sizeof(dmp3_image));
  rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
  if (rc != 0)
  {
    rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
    Serial.print("init got ");
    Serial.println(rc);
    //  INV_MSG(INV_MSG_LEVEL_ERROR, "Initialization failed. Error loading DMP3...");
    return rc;

  }


  /* possible compasses in chip
  *  INV_ICM20948_COMPASS_ID_NONE = 0, 
   INV_ICM20948_COMPASS_ID_AK09911,  
   INV_ICM20948_COMPASS_ID_AK09912,  /**< AKM AK09912 
   INV_ICM20948_COMPASS_ID_AK09916,  /**< AKM AK09916 
   INV_ICM20948_COMPASS_ID_AK08963,  /**< AKM AK08963 
  */

  /* Initialize auxiliary sensors */
  inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);   //AK0991x_SECONDARY_I2C_ADDR); // AK0991x_DEFAULT_I2C_ADDR);

  rc = inv_icm20948_initialize_auxiliary(&icm_device);

  if (rc != 0)
  {
    Serial.print("compass not detected got ");
    Serial.println(rc);
  }
  else
  {

    Serial.println("compass detected");
  }
  icm20948_apply_mounting_matrix();

  icm20948_set_fsr();

  /* re-initialize base state structure */
  inv_icm20948_init_structure(&icm_device);
  return 0;
} //sensor_setup

//---------------------------------------------------------------------

uint64_t inv_icm20948_get_time_us(void)
{
  return millis(); //InvEMDFrontEnd_getTimestampUs();
}

//---------------------------------------------------------------------

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
  switch (sensor)
  {
     case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
       return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
     case INV_SENSOR_TYPE_RAW_GYROSCOPE:
       return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
     case INV_SENSOR_TYPE_ACCELEROMETER:
       return INV_ICM20948_SENSOR_ACCELEROMETER;
     case INV_SENSOR_TYPE_GYROSCOPE:
       return INV_ICM20948_SENSOR_GYROSCOPE;
     case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
       return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
     case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
       return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
     case INV_SENSOR_TYPE_BAC:
       return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
     case INV_SENSOR_TYPE_STEP_DETECTOR:
       return INV_ICM20948_SENSOR_STEP_DETECTOR;
     case INV_SENSOR_TYPE_STEP_COUNTER:
       return INV_ICM20948_SENSOR_STEP_COUNTER;
     case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
       return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
     case INV_SENSOR_TYPE_ROTATION_VECTOR:
       return INV_ICM20948_SENSOR_ROTATION_VECTOR;
     case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
       return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
     case INV_SENSOR_TYPE_MAGNETOMETER:
       return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
     case INV_SENSOR_TYPE_SMD:
       return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
     case INV_SENSOR_TYPE_PICK_UP_GESTURE:
       return INV_ICM20948_SENSOR_FLIP_PICKUP;
     case INV_SENSOR_TYPE_TILT_DETECTOR:
       return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
     case INV_SENSOR_TYPE_GRAVITY:
       return INV_ICM20948_SENSOR_GRAVITY;
     case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
       return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
     case INV_SENSOR_TYPE_ORIENTATION:
       return INV_ICM20948_SENSOR_ORIENTATION;
     case INV_SENSOR_TYPE_B2S:
       return INV_ICM20948_SENSOR_B2S;
     default:
       return INV_ICM20948_SENSOR_MAX;
  }//switch
}//enum sensortyp_conversion


//---------------------------------------------------------------------
// SETUP
//
void setup()
{
  for (size_t i = 0; i < 4; i++)
  {
    pinMode(AD0_PINS[i], OUTPUT);
    digitalWrite(AD0_PINS[i], HIGH);
  }
  Serial.begin(115200);
  while (!Serial)
  {};

  Wire.begin();
  Wire.setClock(400000);

  selectMPU(0);

  struct inv_icm20948_serif icm20948_serif;
  icm20948_serif.context   = 0; /* no need */
  icm20948_serif.read_reg  = idd_io_hal_read_reg;
  icm20948_serif.write_reg = idd_io_hal_write_reg;
  icm20948_serif.max_read  = 1024 * 16; /* maximum number of bytes allowed per serial read */
  icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */

  icm20948_serif.is_spi = interface_is_SPI();

  icm_device.base_state.serial_interface = SERIAL_INTERFACE_I2C;

  inv_icm20948_reset_states(&icm_device, &icm20948_serif);
  inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

  rc = icm20948_sensor_setup();

  if (icm_device.selftest_done && !icm_device.offset_done)
  {
    // If we've run selftes and not already set the offset.
    inv_icm20948_set_offset(&icm_device, unscaled_bias);
    icm_device.offset_done = 1;
  }
  //enable sensors
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_MAGNETOMETER), 1);
} //Setup



void sensor_event(const inv_sensor_event_t *event, void *arg)
{
  /* arg will contained the value provided at init time */
  (void)arg;

  /*
  * Encode sensor event and sent to host over UART through IddWrapper protocol
  */
  /*static DynProtocolEdata_t async_edata; /* static to take on .bss */

  /*static uint8_t async_buffer[256]; 
  uint16_t async_bufferLen;

  async_edata.sensor_id = event->sensor;
  async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
  convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

  if(DynProtocol_encodeAsync(&protocol,
    DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
    async_buffer, sizeof(async_buffer), &async_bufferLen) != 0) {
      goto error_dma_buf;
  }

  DynProTransportUart_tx(&transport, async_buffer, async_bufferLen);
  return;

error_dma_buf:
  INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");
*/
  return;
}

static uint8_t icm20948_get_grv_accuracy(void)
{
  uint8_t accel_accuracy;
  uint8_t gyro_accuracy;

  accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
  gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
  return (min(accel_accuracy, gyro_accuracy));
}
void build_sensor_event_data(void *context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void *data, const void *arg)
{

  float raw_bias_data[6];
  inv_sensor_event_t event;
  (void)context;
  uint8_t sensor_id = convert_to_generic_ids[sensortype];

  memset((void *)&event, 0, sizeof(event));
  event.sensor = sensor_id;
  event.timestamp = timestamp;
  switch (sensor_id)
  {
  case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
    memcpy(raw_bias_data, data, sizeof(raw_bias_data));
    memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
    memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
    memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
    Serial.println("UNCAL gyro data");
    break;
  case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
    memcpy(raw_bias_data, data, sizeof(raw_bias_data));
    memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
    memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
    memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
    Serial.println("Uncalibrated Magnetometer data");
    break;
  case INV_SENSOR_TYPE_GYROSCOPE:
    memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
    memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
    sprintf(eamessage, "Gyro: [%f,%f,%f]", event.data.gyr.vect[0], event.data.gyr.vect[1], event.data.gyr.vect[2]);
    Serial.println(eamessage);

    break;
  case INV_SENSOR_TYPE_GRAVITY:
    memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
    event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
    Serial.println("Gravity data");
    break;
  case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
  case INV_SENSOR_TYPE_ACCELEROMETER:
    memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
    memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));
    sprintf(eamessage, "Accel: [%f,%f,%f]", event.data.acc.vect[0], event.data.acc.vect[0], event.data.acc.vect[0]);
    Serial.println(eamessage);

    break;
  case INV_SENSOR_TYPE_MAGNETOMETER:
    memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
    memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));
    sprintf(eamessage, "Mag: [%f,%f,%f]", event.data.mag.vect[0], event.data.mag.vect[1], event.data.mag.vect[2]);
    Serial.println(eamessage);

    break;
  case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
  case INV_SENSOR_TYPE_ROTATION_VECTOR:
    memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
    memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
    Serial.println("rotation vector");
    break;
  case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
    memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
    event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();
    sprintf(eamessage, "Quaternion: (%f,%f,%f,%f)", event.data.quaternion.quat[0], event.data.quaternion.quat[1], event.data.quaternion.quat[2], event.data.quaternion.quat[3]);
    Serial.println(eamessage);
    break;
  case INV_SENSOR_TYPE_BAC:
    memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
    Serial.println("BAC data...");
    break;
  case INV_SENSOR_TYPE_PICK_UP_GESTURE:
  case INV_SENSOR_TYPE_TILT_DETECTOR:
  case INV_SENSOR_TYPE_STEP_DETECTOR:
  case INV_SENSOR_TYPE_SMD:
    event.data.event = true;
    Serial.println("tilt/step/smd data");
    break;
  case INV_SENSOR_TYPE_B2S:
    event.data.event = true;
    memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
    Serial.println("B2s data");
    break;
  case INV_SENSOR_TYPE_STEP_COUNTER:
    memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
    Serial.println("Step counter data");
    break;
  case INV_SENSOR_TYPE_ORIENTATION:
    //we just want to copy x,y,z from orientation data
    memcpy(&(event.data.orientation), data, 3 * sizeof(float));
    Serial.println("Orientation data");
    break;
  case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
  case INV_SENSOR_TYPE_RAW_GYROSCOPE:
    memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
    Serial.println("RAW accel/gyro data");
    break;
  default:
    return;
  }

  sensor_event(&event, NULL);
}

//-------------------------------------------------------------------------
//LOOP
// just keep polling
void loop()
{
  int rv = inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);
}
