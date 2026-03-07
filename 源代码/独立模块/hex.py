# -*- coding: utf-8 -*-

def generate_mem_files(matrix_size=32):
    
   # 创建所有需要的文件
    files = {
        'a_data.mem': [],
        'b_data.mem': [],
        'c_data.mem': [],
        'expected_results.mem': []
    }
    
    # 生成 A 矩阵数据（行优先）
    for i in range(matrix_size):
        for j in range(matrix_size):
            if i == j:  # 对角线元素
                # 值从 1.0 开始递增（FP32 编码）
                value = 0x3F800000 + i * 0x100000
                files['a_data.mem'].append(f"{value:08x}")
                files['expected_results.mem'].append(f"{value:08x}")
            else:
                files['a_data.mem'].append("00000000")
                files['expected_results.mem'].append("00000000")
    
    # 生成 B 矩阵数据（单位矩阵）
    for i in range(matrix_size):
        for j in range(matrix_size):
            if i == j:  # 对角线元素为 1.0
                files['b_data.mem'].append("3f800000")
            else:
                files['b_data.mem'].append("00000000")
    
    # 生成 C 矩阵数据（全零）
    for _ in range(matrix_size * matrix_size):
        files['c_data.mem'].append("00000000")
    
    # 写入文件
    for filename, data in files.items():
        with open(filename, 'w') as f:
            # 每行一个32位十六进制值
            f.write("\n".join(data))
        
        print(f"Generated {filename} with {len(data)} entries")

# 生成 4x4 矩阵的内存文件
generate_mem_files(4)