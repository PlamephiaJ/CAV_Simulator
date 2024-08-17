'''
Author: Y. Chen moyunyongan@gmail.com
Date: 2024-07-06 10:12:45
LastEditors: Y. Chen moyunyongan@gmail.com
LastEditTime: 2024-07-06 10:50:53
FilePath: /RGBtoMap/RGBtoGreyscale.py
Description: RGB to greyscale
'''

import cv2

def rgb_to_grayscale(image_path, save_path):
    img = cv2.imread(image_path)

    if img is None:
        print(f"Error: Unable to read image from {image_path}")
        return None
    
    
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    print(gray_img)

    first_white_pixel = None
    for i in range(gray_img.shape[0]):
        for j in range(gray_img.shape[1]):
            if gray_img[i,j] != 255:
                first_white_pixel = (-i, -j)
                break
        if first_white_pixel is not None:
            break

    print('Recommended origin pixel coordinate:', first_white_pixel)

    cv2.imwrite(save_path, gray_img)
    
    return gray_img

if __name__ == '__main__':
    input_file_path = 'map/straight.png'
    output_file_path = input_file_path[:-4] + '_greyscale.png'
    if rgb_to_grayscale(input_file_path, output_file_path) is not None:
        print('Finished!')
