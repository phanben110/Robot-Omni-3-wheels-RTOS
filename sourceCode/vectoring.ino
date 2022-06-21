void base1(float magnitude, float theta, int rot, float denta[3])
{  
  //  float denta[3];
  float vel_x, vel_y;
  theta = theta * 0.0174533;              //convert from deg angle to radian
  vel_x = magnitude * cos(theta);
  vel_y = magnitude * sin(theta);

  const float sqrt3o2 = 1.0 * sqrt(3) / 2;
  float v1 = - vel_x + rot;
  float v2 = 0.5 * vel_x - sqrt3o2 * vel_y  + rot;
  float v3 = 0.5 * vel_x + sqrt3o2 * vel_y + rot;

  //  d1 = v1 < 0 ? -1 : 1;
  //  d2 = v2 < 0 ? -1 : 1;
  //  d3 = v3 < 0 ? -1 : 1;

  //  v1 = map(abs(v1), 0, 100, 0, 20);
  //  v2 = map(abs(v2), 0, 100, 0, 20);
  //  v3 = map(abs(v3), 0, 100, 0, 20);
  denta[0] = mapfloat(v1, -100, 100, -20, 20);
  denta[1] = mapfloat(v2, -100, 100, -20, 20);
  denta[2] = mapfloat(v3, -100, 100, -20, 20);

  return denta ;

  //  Serial.println("denta") ;
  //  Serial.println(denta1) ;
  //  Serial.println(denta2) ;
  //  Serial.println(denta3) ;


  //  M1(v1, d1);
  //  M2(v2, d2);
  //  M3(v3, d3);
}
float mapfloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
