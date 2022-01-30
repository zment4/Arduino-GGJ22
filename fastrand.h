uint8_t rand_x = 0;
uint8_t rand_y = 0;
uint8_t rand_z = 0;
uint8_t rand_a = 1;
 
uint8_t fast_rand()
{
  uint8_t t = rand_x ^ (rand_x << 4);
  rand_x = rand_y;
  rand_y = rand_z;
  rand_z = rand_a;
  rand_a = rand_z ^ t ^ (rand_z >> 1) ^ (t << 1);
 
  return rand_a;
}
