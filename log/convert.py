def convert(input_file,output_file):
    # 打开文件  
    with open(input_file, 'r') as file:  
        # 逐行读取文件内容  
        lines = file.readlines()  
          
        # 创建一个空列表来存储结果  
        # data_list = []  
          
        # 遍历每一行，按逗号分割数据并存入列表  
        with open(output_file, 'w') as f:
            for line in lines:
                line_data = line.split(',')  
                # data_list.append(line_data)  
                    # line_data=line_data[0:8]
                # print(line_data)
                for i in range(0,7):
                    f.write(str(line_data[i]))
                    if i==1 or i==2 or i==3:
                        if(float(line_data[i])>1.9 or float(line_data[i])<-1.9):
                            print(line_data[i])
                    f.write(" ")
                # 最后一个后面不能加上空格
                f.write(str(line_data[7]))
                # f.write("\n")
                # print(len(line_data))


        # print(len(data_list))

    # results = []

    # with open('data.csv') as f:
    #   for line in f:
    #     data = line.split(' ')[0:8]
    #     results.append(data)

    # # 保存结果    
    # with open('output.csv', 'w') as f:
    #   for row in results:
    #     f.write(' '.join(row) + '\n')  

input_file="gt_imu.csv"
output_file="groundtruth_tum_room6.txt"


if __name__ == "__main__":
    convert(input_file,output_file)
