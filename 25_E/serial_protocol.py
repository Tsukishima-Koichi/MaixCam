import struct

# 协议数据格式：
# 帧头(0xAA) + 数据域长度 + 数据域 + 长度及数据域数据和校验 + 帧尾(0x55)

class SerialProtocol():
    HEAD = 0xAA
    TAIL = 0x55

    # def __init__(self, serial=None, data_buffer=None) -> None:
    #     # 设置data_buffer默认值（空字节数组）
    #     self.data_buffer = data_buffer if data_buffer is not None else bytearray()
    #     # 设置serial默认值（允许延迟初始化）
    #     self.serial = serial
    def __init__(self) -> None:
        pass

    def _checksum(self, data:bytes) -> int:
        # 计算和校验
        check_sum = 0
        for a in data:
            check_sum = (check_sum + a) & 0xFF
        return check_sum

    def is_valid(self, raw_data:bytes) -> tuple:
        # 判断数据是否有效
        bytes_redundant = 0
        index = 0

        for a in raw_data:
            if a != SerialProtocol.HEAD:
                index += 1
            else:
                break

        bytes_redundant = index

        if len(raw_data[index:]) < 3:
            return (-1, bytes_redundant)

        payload_len = struct.unpack('<H', raw_data[index+1:index+3])[0]
        if len(raw_data)-bytes_redundant < payload_len+5:
            return (-2, bytes_redundant)

        if raw_data[index+3+payload_len+1] != SerialProtocol.TAIL or self._checksum(raw_data[index+1:index+3+payload_len]) != raw_data[index+3+payload_len]:
            return (-3, bytes_redundant)
        else:
            return (0, bytes_redundant)
    
    def length(self, raw_data:int) -> int:
        if len(raw_data) < 5 or raw_data[0] != SerialProtocol.HEAD or raw_data[-1] != SerialProtocol.TAIL:
            return -1

        payload_len = struct.unpack('<H', raw_data[1:3])[0]
        return (3+payload_len+2)

    def encode(self, payload:bytes) -> bytes:
        # 编码数据负载部分，添加帧头帧尾校验等部分
        frame = bytearray()
        frame.append(SerialProtocol.HEAD)
        frame.extend(struct.pack('<H', len(payload)))
        frame.extend(payload)
        frame.append(self._checksum(frame[1:]))
        frame.append(SerialProtocol.TAIL)
        return bytes(frame)

    def decode(self, raw_data:bytes) -> bytes:
        # 解码出数据负载部分
        if len(raw_data) < 5 or raw_data[0] != SerialProtocol.HEAD or raw_data[-1] != SerialProtocol.TAIL:
            return bytes()
        payload_len = struct.unpack('<H', raw_data[1:3])[0]
        return raw_data[3:3+payload_len]

    # def proto_recv(self):
    #     if self.serial is None:
    #         return None        

    #     # 获取待读取长度（从协议层获取预期长度）
    #     length = self.length(self.data_buffer) if len(self.data_buffer) > 0 else 0
        
    #     if length > 0:
    #         data = self.serial.read(length)
    #         self.data_buffer += data
            
    #         # 协议有效性检查与冗余字节处理
    #         rc, bytes_redundant = self.is_valid(self.data_buffer)
    #         if bytes_redundant > 0:
    #             self.data_buffer = self.data_buffer[bytes_redundant:]
            
    #         # 协议解码与结果提取
    #         if rc >= 0:
    #             result = self.decode(self.data_buffer)
    #             if len(result) == 16:
    #                 x0, y0, x1, y1 = struct.unpack('<iiii', result)
    #                 # print(f'{x0},{y0},{x1},{y1}')
    #                 return self.data_buffer, (x0, y0, x1, y1)  # 返回缓冲区和解析结果
        
    #     # 更新缓冲区（移除已处理数据包）
    #     packet_length = self.length(self.data_buffer)
    #     self.data_buffer = self.data_buffer[packet_length:]
    
    #     return self.data_buffer, None  # 返回更新后的缓冲区和空结果

if __name__ == '__main__':
    payload = 'hello'
    proto = SerialProtocol()

    encoded = proto.encode(payload.encode())
    print(encoded.hex())

    encoded = bytes([0x01, 0x02]) + encoded
    valid = proto.is_valid(encoded)
    print(valid)

    decoded = encoded[valid[1]:]
    decoded = proto.decode(decoded)
    print(decoded.decode())