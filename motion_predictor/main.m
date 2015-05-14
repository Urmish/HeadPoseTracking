
clear all 
clc
predictor_length = 8
% Input matrix
input = csvread('Transformation_dump.csv');
[num_frames num_features] = size(input)


% Apply standard deviation to the values
arr_col =[];
std_col =[];

for i = 1:num_features
    arr_col(i,:)=input(:,i)';
    std_col(i) = std(arr_col(i,:));
end



% Generate T
T = [];

for i = 2:num_frames
    T(i,:)= input(i,:)-input(i-1,:);  
end

% Generate Q (or X in Normal equation)

one_array = ones(1,num_features);

set = [];
row_no = 1;
output_set = [];
for i = (predictor_length+1):num_frames
    
    if(predictor_length == 8)
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)',T(i-2,:)',T(i-3,:)',T(i-4,:)',T(i-5,:)',T(i-6,:)',T(i-7,:)');
    elseif(predictor_length == 7)
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)',T(i-2,:)',T(i-3,:)',T(i-4,:)',T(i-5,:)',T(i-6,:)');
    elseif(predictor_length == 6)
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)',T(i-2,:)',T(i-3,:)',T(i-4,:)',T(i-5,:)');    
    elseif(predictor_length == 5)
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)',T(i-2,:)',T(i-3,:)',T(i-4,:)');
    elseif(predictor_length == 4)
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)',T(i-2,:)',T(i-3,:)');
    elseif(predictor_length == 3)
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)',T(i-2,:)');
    elseif (predictor_length ==2) 
        set(:,:,row_no) = horzcat(one_array',T(i-1,:)');    
    else
        fprintf('Unknown predictor length\n')
        break
    end
    
    output_set(:,:,row_no)=T(i,:)';
    row_no = row_no + 1;
    
end

total_rows = row_no -1 ;


% Do vertical concatenation

Q = [];
P =[];

Q=vertcat(set(:,:,1),set(:,:,2));
P=vertcat(output_set(:,:,1),output_set(:,:,2));
for i = 3:total_rows
    Q = vertcat(Q,set(:,:,i));
    P = vertcat(P,output_set(:,:,i));
end


% Now solve for the values of a

A = (pinv(Q'*Q))*(Q')*(P)

% Calculate mean square error

mse = 0;

for i = 1:total_rows
    actual_output = output_set(:,:,i);
    predicted_output = set(:,:,i)*A;
    
    D = abs(actual_output-predicted_output).^2;
    mse = mse+sum(D(:));  
end

mse = mse/(num_features*total_rows)




%=======================================================================
%======== Method 2 : Taking different features separately ==============
%======================================================================
P1=[];
P2=[];
P3=[];
P4=[];
P5=[];
P6=[];

Q1=[];
Q2=[];
Q3=[];
Q4=[];
Q5=[];
Q6=[];


for i = 1:total_rows
    Q1(i,:)=set(1,:,i);
    Q2(i,:)=set(2,:,i);
    Q3(i,:)=set(3,:,i);
    Q4(i,:)=set(4,:,i);
    Q5(i,:)=set(5,:,i);
    Q6(i,:)=set(6,:,i);
    
    
    P1(i,:) = output_set(1,:,i);
    P2(i,:) = output_set(2,:,i);
    P3(i,:) = output_set(3,:,i);
    P4(i,:) = output_set(4,:,i);
    P5(i,:) = output_set(5,:,i);
    P6(i,:) = output_set(6,:,i);
end


A1 = (pinv(Q1'*Q1))*(Q1')*(P1);
A2 = (pinv(Q2'*Q2))*(Q2')*(P2);
A3 = (pinv(Q3'*Q3))*(Q3')*(P3);
A4 = (pinv(Q4'*Q4))*(Q4')*(P4);
A5 = (pinv(Q5'*Q5))*(Q5')*(P5);
A6 = (pinv(Q6'*Q6))*(Q6')*(P6);


actual_output1 = P1;
predicted_output1 = Q1*A1;
D1 = abs(actual_output1-predicted_output1).^2;
mse1=sum(D1(:))/total_rows

actual_output2 = P2;
predicted_output2 = Q2*A2;
D2 = abs(actual_output2-predicted_output2).^2;
mse2=sum(D2(:))/total_rows

actual_output3 = P3;
predicted_output3 = Q3*A3;
D3 = abs(actual_output3-predicted_output3).^2;
mse3=sum(D3(:))/total_rows 

actual_output4 = P4;
predicted_output4 = Q4*A4;
D4 = abs(actual_output4-predicted_output4).^2;
mse4=sum(D4(:))/total_rows

actual_output5 = P5;
predicted_output5 = Q5*A5;
D5 = abs(actual_output5-predicted_output5).^2;
mse5=sum(D5(:))/total_rows 

actual_output6 = P6;
predicted_output6 = Q6*A6;
D6 = abs(actual_output6-predicted_output6).^2;
mse6=sum(D6(:))/total_rows

avg = (mse1+mse2+mse3+mse4+mse5+mse6)/6
