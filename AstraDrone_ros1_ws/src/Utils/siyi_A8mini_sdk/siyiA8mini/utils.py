class utils:
    
    def int_to_hex_array_uint16(self,num):
        # 将整数转换为16位的2的补码，使用 'h' 格式编码 (short)，小端模式
        hex_bytes = num.to_bytes(2, byteorder='little', signed=True)
    
        # 将每个字节转换为十六进制表示
        #hex_array = [f"0x{byte:02x}" for byte in hex_bytes]
        return [(hex_bytes[0]), (hex_bytes[1])]
    
    def int_to_hex_array_uint8(self,num):
        # 将整数转换为8位的有符号数，使用小端模式
        hex_bytes = num.to_bytes(1, byteorder='little', signed=True)
    
        # 返回字节的值
        return [hex_bytes[0]]
        




