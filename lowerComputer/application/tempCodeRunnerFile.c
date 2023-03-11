static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll)
{
    //yaw轴数值转化
    date32_to_date8_t yaw_temp;
    //pitch轴数值转化
    date32_to_date8_t pitch_temp;
    //roll轴数据转化
    date32_to_date8_t roll_temp;
    //数据起始
    date32_to_date8_t head1_temp;
    date32_to_date8_t head2_temp;
    //中止位
    date32_to_date8_t end_temp;
    //模式转换
    date32_to_date8_t switch_temp;

    //赋值数据
    yaw_temp.uint32_val = yaw * DOUBLE_;
    pitch_temp.uint32_val = pitch * DOUBLE_;
    roll_temp.uint32_val = roll * DOUBLE_;
    head1_temp.uint32_val = 0x34;
    head2_temp.uint32_val = 0x43;
    end_temp.uint32_val = 0x0A;//"/n"
    switch_temp.uint32_val = 1;

    for (int i = 3; i >= 0; i--)
    {
        int j = -(i - 3);//数据位置转换
        buf[0 * 4 + j] = head1_temp.uin8_value[i];
        buf[1 * 4 + j] = head2_temp.uin8_value[i];
        buf[2 * 4 + j] = yaw_temp.uin8_value[i];
        buf[3 * 4 + j] = pitch_temp.uin8_value[i];
        buf[4 * 4 + j] = roll_temp.uin8_value[i];
        buf[5 * 4 + j] = switch_temp.uin8_value[i]; 
        buf[6 * 4 + j] = end_temp.uin8_value[i];
    }
}

