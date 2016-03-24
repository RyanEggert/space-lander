// DEVICE INFORMATION
#define LOCAL_ADDR 0x01

// Quad Encoder
typedef struct {
    unsigned char a_curr;
    unsigned char a_prev;
    unsigned char b_curr;
    unsigned char b_prev;
    uint32_t count;
    _PIN* A;
    _PIN* B; 

} QuadEncoder;

QuadEncoder qenc;
int8_t quad_lut [] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

