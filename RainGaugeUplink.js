function decodeUplink(input) {
  var rain = input.bytes[0];
  
  var temperature = (input.bytes[2] << 0) | (input.bytes[1] << 8);
  
  var humidity = input.bytes[3];
  
  var voltage_adc = (input.bytes[4] << 8) | (input.bytes[5] << 0);

  return {
    data: {
      temperature: temperature/100,
      humidity: humidity,
      rain: rain*0.3,
      voltage: voltage_adc/1000
    }
  };

}