
def(false_detections):
    pixel_list = []    # extracting pixel values and its coordinates for each false positive detections
    for i in false_detections:  #false_detections is a list of array containing the bounding box coordinates (x1,y1,x2,y2)

        x1 = i[0][0]     #bounding box coordinates for false positive detections
        x2 = i[0][2]
        y1 = i[0][1]
        y2 = i[0][3]

        num_col = 12  # default
        num_row = 3

        index_top_row = range(1, num_row + 1)
        index_bottom_row = range(1, num_row)

        check_num_row = range(3, 10)
        check_num_col = range(10, 30)
        if num_row not in check_num_row or num_col not in check_num_col:
            raise ValueError("Pixel Rows and Columns Selection not Valid. Please adjust the number of rows or columns")

        if abs(y1 - y2) >= abs(x1 - x2):
            top = abs(x2 - x1) / (num_row + 1)  # num_col
            bottom = abs(x2 - x1) / num_row
            ver = abs(y1 - y2) / (num_col + 1)  # num_row
            y = y1
            y_list = []
            for i in range(num_col):
                y = round(y + ver)
                y_list.append(y)
            i = 1
            # pixel_list = []
            for y in y_list:
                if (i % 2) == 1:  # then i is odd % is remainder module
                    for index in index_top_row:
                        pixel_list.append((y, round(x1 + index * top)))
                else:
                    for index in index_bottom_row:
                        pixel_list.append((y, round(x1 + index * bottom)))
                i += 1

        else:
            top = abs(y1 - y2) / (num_row + 1)
            bottom = abs(y1 - y2) / num_row
            hor = abs(x1 - x2) / (num_col + 1)

            x = x1
            x_list = []
            for i in range(num_col):
                x = round(x + hor)
                x_list.append(x)
            i = 1
            # pixel_list = []

            for x in x_list:
                if (i % 2) == 1:
                    for index in index_top_row:
                        pixel_list.append((round(y1 + index * top), x))
                else:
                    for index in index_bottom_row:
                        pixel_list.append((round(y1 + index * bottom), x))

                i += 1

    len(pixel_list)
    return (pixel_list)

