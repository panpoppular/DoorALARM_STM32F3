# FinalTest2

คำสั่งที่ใช้ได้:

Ax เปิด Alarm ตัวที่ x

Dx ปิด Alarm ตัวที่ x

S0 ปิด Thread UART สำหรับส่งข้อมูล Sensor (ค่าเริ่มต้นคือปิด)

S1 เปิด Thread  UART สำหรับส่งข้อมูล Sensor

ข้อมูลที่ส่งจาก UART จะส่งเป็นข้อมูล 8 Byte โดยที่แต่ละ Byte คือ Sensor 1 ตัว

สามารถรับได้โดยใช้คำสั่ง Serial.readBytes()
