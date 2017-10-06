function [I,row,col] = ReadImage()
src = imread('Lower-Abdomen.jpg');
I = rgb2gray(src);
[row, col]=size(I);
end