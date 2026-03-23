%% saban_input_pid_wheels.m (Đã tích hợp G(s) và Dừng 2s)
clc; clear; close all;
%% 1. Định nghĩa Đường đua (Điểm và Đoạn)
%--- Định nghĩa các điểm tọa độ ---
s = sqrt(2);
A = [3000, 0]; B = [500, 0]; D = [500, 1000]; E = [1800 - 800*s, 1000];
F = [1800 - 400*s, 1800 - 400*s]; H = [1800, 1500]; K = [3000,1500]; S = [1000,0];
G = [1800 - 400*s, 200 + 400*s]; I = [1800, 500]; Lp = [3000, 500];
R_arc  = 800; R_arc1 = 500; trackWidth = 26;
%--- Phân chia đường đua thành các đoạn ---
segments_common = {};
segments_common{end+1} = struct('type','line','P1',A,'P2',B,'width',trackWidth);
segments_common{end+1} = struct('type','arcR','P1',B,'P2',D,'R',R_arc1,'width',trackWidth,'cw', true);
segments_common{end+1} = struct('type','line','P1',D,'P2',E,'width',trackWidth);
segments_red = {};
segments_red{end+1} = struct('type','arcR','P1',E,'P2',F,'R',R_arc,'width',trackWidth,'cw',false);
segments_red{end+1} = struct('type','arcR','P1',F,'P2',H,'R',R_arc,'width',trackWidth,'cw',true);
segments_red{end+1} = struct('type','line','P1',H,'P2',K,'width',trackWidth);
segments_green = {};
segments_green{end+1} = struct('type','arcR','P1',E,'P2',G,'R',R_arc,'width',trackWidth,'cw',true);
segments_green{end+1} = struct('type','arcR','P1',G,'P2',I,'R',R_arc,'width',trackWidth,'cw',false);
segments_green{end+1} = struct('type','line','P1',I,'P2',Lp,'width',trackWidth);
points = struct('A',A,'B',B,'D',D,'E',E,'F',F,'H',H,'K',K,'S',S,'G',G,'I',I,'Lp',Lp);
%% 2. Thiết lập Đồ họa Ban đầu (Vẽ Đường đua)
%--- Vẽ đường đua cơ sở ---
figure('Name','Robot dò line với động học'); hold on; axis equal; grid on;
drawTrackBlack([segments_common, segments_red, segments_green]);
drawPoints(points);
title('Đường đua đen — Robot với giới hạn gia tốc');
activeSegments = segments_common;
%--- Đồ họa (Handles) ---
hCar = patch(0,0,'r'); hWheelL = patch(0,0,'k'); hWheelR = patch(0,0,'k');
hSensors = plot(NaN,NaN,'bo','MarkerFaceColor','b');
hTrailPre  = plot(NaN,NaN,'b-','LineWidth',1.5); hTrailPost = plot(NaN,NaN,'-','LineWidth',1.5);
cargoPlot = [];
%% 3. Thông số Mô phỏng và Robot
%--- Thông số Robot ---
L = 178;   % Khoảng cách giữa hai bánh xe (mm)
W = 140;   % Chiều rộng thân robot (mm)
H_robot = 218.5;  % Chiều dài thân robot (mm)
% Đặt v_base < v_max để có "Headroom" (khoảng trống) cho điều khiển
v_base = 500;     % Tốc độ mục tiêu (mm/s)
v_max = 600;      % Tốc độ TỐI ĐA
v_min = 0;
dt = 0.025; % Bước thời gian mô phỏng (s)
T  = 45;    % TỔNG THỜI GIAN mô phỏng (s)
N  = round(T / dt); % Tự động tính tổng số bước lặp
%--- Khởi tạo trạng thái Robot ---
x = A(1); y = A(2); theta = -pi-pi/18; % Cố tình tạo 1 góc lệch ban đầu (10 độ)
%--- Thông số Cảm biến ---
d_sensor = 200; sensor_gap = 12;
weights = [-20 -10 0 10 20]; % Dải sai số chuẩn hóa err = [-20, 20]
max_sense_dist = trackWidth/2;
%% 4. HỆ SỐ PID
%--- Hệ số PID vòng ngoài (Điều khiển hướng) ---
Kp = 3;  
Ki = 0.02; 
Kd = 0.;
err_int = 0; prev_err = 0; err = 0;
%--- Hệ số PID vòng trong (Điều khiển tốc độ bánh) ---
Kp_w = 0.224; 
Ki_w = 14.4;
Kd_w = 0;     % Chúng ta đã thiết kế bộ PI
vl_actual = 0; vr_actual = 0;
vl_err_int = 0; vr_err_int = 0;
vl_prev_err = 0; vr_prev_err = 0;
%% 5. Khởi tạo Trạng thái và Bộ nhớ (Logging)
%--- Các biến trạng thái ---
pickedCargo = false; cargo = ''; branched = false; passedS = false;
% === Thêm biến trạng thái cho việc Dừng xe ===
is_stopping = false; % Cờ báo robot đang trong trạng thái dừng
stop_timer = 0.0;    % Bộ đếm thời gian đã dừng
stop_duration = 2.0; % Robot sẽ dừng trong 2 giây

%--- Lưu trữ vệt đường đi ---
trailX_pre = []; trailY_pre = []; trailX_post = []; trailY_post = [];
%--- Khởi tạo bộ nhớ lưu trữ dữ liệu (Logging) ---
t_log      = zeros(1,N); vl_log = zeros(1,N); vr_log = zeros(1,N);
vl_cmd_log = zeros(1,N); vr_cmd_log = zeros(1,N); v_rb_actual_log = zeros(1,N);
err_log    = zeros(1,N); signals_log = zeros(N, 5); d_center_log = zeros(1,N);
u_log      = zeros(1,N);

% === Rời rạc hóa Hàm truyền Động cơ G(s) ===
fprintf('\n--- Đang rời rạc hóa hàm truyền động cơ G(s) ---\n');
s = tf('s');
Gp_cont = 43.1564 / (s + 25.668); % Hàm truyền động cơ
Gp_disc = c2d(Gp_cont, dt, 'zoh'); 
[num, den] = tfdata(Gp_disc, 'v');
b1_z = num(2); 
a1_z = den(2); 
fprintf('Hàm truyền G(s) đã được rời rạc hóa thành:\n');
fprintf('y(k) = -a1*y(k-1) + b1*u(k-1)\n');
fprintf('Với: a1=%.4f, b1=%.4f\n\n', a1_z, b1_z);
% Khởi tạo "bộ nhớ" (các trạng thái trước đó) cho vòng lặp
vl_actual_prev = 0; 
vr_actual_prev = 0; 
wL_cmd_prev = 0;    
wR_cmd_prev = 0;    

%% 6. VÒNG LẶP MÔ PHỎNG CHÍNH
for k = 1:N
    t_now = (k-1)*dt;
    t_log(k) = t_now;
    
    % ===================================================================
    % === LOGIC DỪNG 2 GIÂY (MỚI) ===
    % ===================================================================
    if is_stopping
        stop_timer = stop_timer + dt; % Bắt đầu đếm thời gian
        
        if stop_timer >= stop_duration
            % --- Đã dừng đủ 2 giây ---
            is_stopping = false; % Tắt cờ
            disp('Đã dừng 2 giây, tiếp tục di chuyển.');
            
            % Reset bộ nhớ PID để xe không bị "vọt" ga
            err_int = 0; prev_err = 0; err = 0;
            vl_err_int = 0; vr_err_int = 0;
            vl_prev_err = 0; vr_prev_err = 0;
            % (Để vòng lặp chạy bình thường, không 'continue')
            
        else
            % --- Vẫn đang trong 2 giây chờ ---
            
            % GHI LOG TRẠNG THÁI ĐỨNG YÊN
            % (Đây là bước quan trọng để vẽ đồ thị)
            vl_log(k) = 0;
            vr_log(k) = 0;
            v_rb_actual_log(k) = 0;
            err_log(k) = 0;
            u_log(k) = 0;
            vl_cmd_log(k) = 0;
            vr_cmd_log(k) = 0;
            if k > 1, d_center_log(k) = d_center_log(k-1); end
            
            % Giữ nguyên vị trí (không cập nhật x,y,theta)
            % bằng cách bỏ qua phần còn lại của vòng lặp
            continue; 
        end
    end
    % === KẾT THÚC LOGIC DỪNG ===
    
    % --- Tính toán vị trí 5 cảm biến ---
    [Xs,Ys] = sensorPositions(x,y,theta,d_sensor,sensor_gap);
    
    % --- Giai đoạn: Nhặt hàng tại điểm S (ĐÃ SỬA) ---
    if ~pickedCargo && norm([x,y]-S) < 60
        
        % 1. Dừng mô phỏng (lần cuối) để chờ người dùng chọn
        choice = questdlg('Chọn màu hàng:', 'Chọn hàng hóa', 'Red', 'Green', 'Red');
        switch choice
            case 'Red', cargo = 'red';
            case 'Green', cargo = 'green';
            otherwise, cargo = '';
        end
        if isempty(cargo), disp('Hủy chọn hàng. Dừng.'); break; end
        
        % 2. KÍCH HOẠT TRẠNG THÁI DỪNG
        is_stopping = true; % Bật cờ
        stop_timer = 0.0;   % Bắt đầu đếm
        pickedCargo = true; % Đánh dấu đã nhặt hàng
        disp(['Đã nhặt hàng tại S: ', cargo, '. Bắt đầu dừng 2 giây...']);
        
        % 3. Reset vận tốc & bộ nhớ G(z) về 0
        vl_actual = 0; vr_actual = 0;
        vl_actual_prev = 0; vr_actual_prev = 0;
        wL_cmd_prev = 0; wR_cmd_prev = 0;
        
        % 4. Cập nhật đồ họa
        cargoPlot = plot(x, y, 's', 'MarkerSize',14, 'MarkerFaceColor', cargo, 'MarkerEdgeColor','k');
        drawnow;
        if passedS, set(hTrailPost,'Color',cargo); end
        
        % 5. GHI LOG điểm dừng ĐẦU TIÊN
        vl_log(k) = 0; vr_log(k) = 0;
        v_rb_actual_log(k) = 0;
        err_log(k) = 0; u_log(k) = 0;
        vl_cmd_log(k) = 0; vr_cmd_log(k) = 0;
        if k > 1, d_center_log(k) = d_center_log(k-1); end

        % 6. Bỏ qua phần còn lại, để vòng lặp (k+1) bắt đầu logic dừng
        continue; 
    end
    
    % --- Kiểm tra nếu đã đến đích ---
    if branched
        if strcmp(cargo,'red'), goal = K; else, goal = Lp; end
        if norm([x,y]-goal) <= 18
            disp(['Đã đến đích tại nhánh ', cargo, '!']);
            v_base = 0; pause(2); break;
        end
    end
    
    % --- Đọc tín hiệu cảm biến dò line ---
    signals = zeros(1,5);
    if ~branched
        segments_to_scan = [segments_common, segments_red, segments_green];
    else
        segments_to_scan = activeSegments;
    end
    for i=1:5
        P_sensor = [Xs(i), Ys(i)];
        d_center = getDistanceToCenterline(P_sensor, segments_to_scan);
        if i == 3, d_center_log(k) = d_center; end
        if d_center < max_sense_dist
            val = 1.0 - (d_center / max_sense_dist);
            signals(i) = max(0, val);
        else
            signals(i) = 0.0;
        end
    end
    signals_log(k, :) = signals;
    
    % === LOGIC RẼ NHÁNH THEO TÍN HIỆU CẢM BIẾN [a b 0 c d] ===
    thresh_on = 0.05; thresh_off = 0.8;
    sees_left = signals(1) > thresh_on || signals(2) > thresh_on;
    sees_right = signals(4) > thresh_on || signals(5) > thresh_on;
    sees_center_gap = signals(3) < thresh_off;
    is_at_fork = sees_left && sees_right && sees_center_gap;
    
    % === Xử lý Tốc độ tại Ngã rẽ và Cung tròn ===
    if ~branched && pickedCargo && is_at_fork
        % 1. Đang ở ngã ba -> Ghi đè 'err' VÀ 'v_dyn'
        branched = true;
        if strcmp(cargo, 'red')
            err = -5; activeSegments = [segments_common segments_red];
            disp('NGÃ BA [a b 0 c d]! Chọn ĐỎ -> Rẽ Trái.');
        else % 'green'
            err = 5; activeSegments = [segments_common segments_green];
            disp('NGÃ BA [a b 0 c d]! Chọn XANH -> Rẽ Phải.');
        end
        err_int = 0; prev_err = err;
        if pickedCargo, set(hTrailPost,'Color',cargo); end
        
    else
        % 2. Không phải ngã ba -> Tính 'err' VÀ 'v_dyn' bình thường
        if sum(signals) > 0.01
            err = sum(weights.*signals)/sum(signals);
        else
            err = prev_err; % Giữ lỗi cũ nếu mất vạch
        end
        % --- Logic giảm tốc độ trên cung tròn ---
        P_mid = [Xs(3), Ys(3)]; % Cảm biến giữa
        d_min = inf;
        current_seg_type = 'line'; % Giả định
        for i_seg = 1:length(segments_to_scan)
            seg = segments_to_scan{i_seg};
            d_seg = getDistanceToCenterline(P_mid, {seg}); 
            if d_seg < d_min
                d_min = d_seg;
                current_seg_type = seg.type;
            end
        end
        
        % Áp dụng logic tốc độ
        if strcmp(current_seg_type, 'arcR')
            % (Logic này đang trống)
        else
            % (Logic này đang trống)
        end
    end
    err_log(k) = err;
    
    % --- Bộ điều khiển PID vòng ngoài (Dò line) ---
    err_int = err_int + err*dt;
    err_der = (err - prev_err)/dt;
    u = Kp*err + Ki*err_int + Kd*err_der;
    u_log(k) = u;
    prev_err = err;
    
    % --- Tính tốc độ mục tiêu động (Dynamic Speed Scaling) ---
    scale = max(0.5, 1 - abs(err)/20); 
    v_dyn = v_base * scale;
    
    % --- Tính toán và giới hạn lệnh tốc độ bánh xe ---
    vl_cmd = min(max(v_dyn - u, -v_max), v_max);
    vr_cmd = min(max(v_dyn + u, -v_max), v_max);
    vl_cmd_log(k) = vl_cmd; vr_cmd_log(k) = vr_cmd;
    
    % === VÒNG PID TRONG: Điều khiển tốc độ thực tế từng bánh ===
    errL = vl_cmd - vl_actual; vl_err_int = vl_err_int + errL*dt;
    vl_err_der = (errL - vl_prev_err)/dt;
    wL_cmd = min(max(Kp_w*errL + Ki_w*vl_err_int + Kd_w*vl_err_der, -v_max), v_max);
    vl_prev_err = errL;
    errR = vr_cmd - vr_actual; vr_err_int = vr_err_int + errR*dt;
    vr_err_der = (errR - vr_prev_err)/dt;
    wR_cmd = min(max(Kp_w*errR + Ki_w*vr_err_int + Kd_w*vr_err_der, -v_max), v_max);
    vr_prev_err = errR;
    
    % === Mô phỏng Động cơ từ G(z) ===
    % Phương trình: y(k) = -a1*y(k-1) + b1*u(k-1)
    
    % Bánh Trái (Left Wheel)
    vl_actual = -a1_z*vl_actual_prev + b1_z*wL_cmd_prev;
    
    % Bánh Phải (Right Wheel)
    vr_actual = -a1_z*vr_actual_prev + b1_z*wR_cmd_prev;
    
    % Giới hạn vận tốc
    vl_actual = min(max(vl_actual, -v_max), v_max);
    vr_actual = min(max(vr_actual, -v_max), v_max);
    
    % Cập nhật trạng thái (k-1) cho vòng lặp tiếp theo
    vl_actual_prev = vl_actual; % y(k)
    vr_actual_prev = vr_actual; % y(k)
    wL_cmd_prev = wL_cmd;       % u(k)
    wR_cmd_prev = wR_cmd;       % u(k)
    
    vl_log(k) = vl_actual; vr_log(k) = vr_actual;
    v_rb_actual = (vl_actual + vr_actual)/2;
    v_rb_actual_log(k) = v_rb_actual;
    
    % --- Cập nhật vị trí và góc Robot ---
    w = (vr_actual - vl_actual) / L;
    x = x + v_rb_actual*cos(theta)*dt;
    y = y + v_rb_actual*sin(theta)*dt;
    theta = theta + w*dt;
    
    % --- Vẽ vệt đường robot di chuyển (từ cảm biến giữa) ---
    x_err = Xs(3); 
    y_err = Ys(3);
    if ~passedS
        trailX_pre(end+1)=x_err; trailY_pre(end+1)=y_err;
    else
        trailX_post(end+1)=x_err; trailY_post(end+1)=y_err;
    end
    
    % --- Kiểm tra khi robot vượt qua điểm S ---
    if ~passedS && norm([x,y]-S)<50
        passedS = true;
        disp('Đã qua điểm S: đổi màu vệt đường theo hàng hóa');
        if ~isempty(trailX_pre)
            trailX_post=[trailX_post,trailX_pre(end)];
            trailY_post=[trailY_post,trailY_pre(end)];
        end
        if pickedCargo, set(hTrailPost,'Color',cargo); end
    end
    
    % --- Cập nhật đồ họa ---
    set(hTrailPre,'XData',trailX_pre,'YData',trailY_pre);
    set(hTrailPost,'XData',trailX_post,'YData',trailY_post);
    [Xb,Yb] = transformRect(x,y,theta,W,H_robot);
    nx=-sin(theta); ny=cos(theta);
    [Xwl,Ywl]=transformRect(x+nx*L/2,y+ny*L/2,theta,50,20);
    [Xwr,Ywr]=transformRect(x-nx*L/2,y-ny*L/2,theta,50,20);
    set(hCar,'XData',Xb,'YData',Yb);
    set(hWheelL,'XData',Xwl,'YData',Ywl);
    set(hWheelR,'XData',Xwr,'YData',Ywr);
    set(hSensors,'XData',Xs,'YData',Ys);
    if pickedCargo, set(cargoPlot, 'XData', x, 'YData', y); end
    drawnow;
end % Kết thúc vòng lặp mô phỏng chính
%% 7. Cắt ngắn dữ liệu Log
validLen = find(t_log>0,1,'last');
if isempty(validLen), validLen = length(t_log); end
t_log = t_log(1:validLen); vl_log = vl_log(1:validLen); vr_log = vr_log(1:validLen);
vl_cmd_log = vl_cmd_log(1:validLen); vr_cmd_log = vr_cmd_log(1:validLen);
v_rb_actual_log = v_rb_actual_log(1:validLen); err_log = err_log(1:validLen);
signals_log = signals_log(1:validLen, :); d_center_log = d_center_log(1:validLen);
u_log = u_log(1:validLen);
%% 8. VẼ BIỂU ĐỒ KẾT QUẢ
figure('Name', 'So sánh tốc độ lệnh và tốc độ thực tế');
subplot(2, 1, 1);
plot(t_log, vl_log, 'b', 'LineWidth', 0.5); hold on;
plot(t_log, vl_cmd_log, 'k--', 'LineWidth', 0.5);
xlabel('Thời gian (s)'); ylabel('Vận tốc (mm/s)'); title('Bánh Trái: Lệnh vs Thực tế');
legend('v_L thực tế', 'v_L lệnh', 'Location', 'best'); grid on;
subplot(2, 1, 2);
plot(t_log, vr_log, 'r', 'LineWidth', 0.5); hold on;
plot(t_log, vr_cmd_log, 'k--', 'LineWidth', 0.5);
xlabel('Thời gian (s)'); ylabel('Vận tốc (mm/s)'); title('Bánh Phải: Lệnh vs Thực tế');
legend('v_R thực tế', 'v_R lệnh', 'Location', 'best'); grid on;

figure('Name', 'So sánh vận tốc hai bánh');
plot(t_log,vl_log,'b',t_log,vr_log,'r','LineWidth',0.5);
xlabel('Thời gian (s)'); ylabel('Vận tốc (mm/s)');
legend('v_L thực tế','v_R thực tế'); title('So sánh vận tốc hai bánh'); grid on;

figure('Name', 'Vận tốc thẳng thực tế');
plot(t_log,v_rb_actual_log,'k','LineWidth',0.5);
xlabel('Thời gian (s)'); ylabel('v_{robot thực tế} (mm/s)');
title('Vận tốc thẳng thực tế của Robot'); grid on;

figure('Name', 'Sai số ngang Cảm biến Giữa');
signed_d_center_log = d_center_log .* arrayfun(@(x) sign(x) + (x==0), err_log);
plot(t_log, signed_d_center_log, 'c', 'LineWidth', 0.5); % Màu cyan
hold on;
plot(t_log, zeros(size(t_log)), 'k-', 'LineWidth', 0.5, 'DisplayName','Tâm vạch (0 mm)');
half_width = trackWidth / 2;
plot(t_log, ones(size(t_log)) * half_width, 'k--','DisplayName','Mép vạch');
plot(t_log, -ones(size(t_log)) * half_width, 'k--','DisplayName','_Mép vạch');
title('Sai số Ngang Ước Lượng (mm) - Chỉ từ Cảm biến Giữa');
xlabel('Thời gian (s)');
ylabel({'Sai số Ngang (mm)'; '(>0: Lệch Trái, <0: Lệch Phải)'});
legend('Sai số CB Giữa (mm)', 'Tâm vạch', 'Mép vạch', 'Location', 'best');
grid on;
ylim([-half_width*1.5, half_width*1.5]);
%% 9. Phân tích Đáp ứng (Robust Response Analysis)
t = t_log; v = v_rb_actual_log; v_ref = v_base;
win = max(1,min(floor(1/dt),floor(0.1*length(v))));
final_est = mean(v(max(1,length(v)-win+1):length(v)));
if final_est < 0.5*v_ref, final_used = v_ref; else, final_used = final_est; end
peak_value = max(v); PO = (peak_value - final_used)/abs(final_used)*100;
if final_used > 0
    t10 = find(v >= 0.1*final_used,1,'first'); t90 = find(v >= 0.9*final_used,1,'first');
    if ~isempty(t10)&&~isempty(t90), trise = t(t90)-t(t10); else, trise = NaN; end
else, trise = NaN; end
tol = 0.05*abs(final_used); settle_idx = NaN;
for i=1:length(v)
    if all(abs(v(i:end)-final_used)<=tol), settle_idx=i; break; end
end
if ~isnan(settle_idx), t_settle=t(settle_idx); else, t_settle=NaN; end
idx_base=find(v>=0.99*v_ref,1,'first');
if ~isempty(idx_base), t_to_base=t(idx_base); else, t_to_base=NaN; end

% Tính hằng số thời gian phân tích từ G(s)
% G(s) = 27.7231 / (s + 18.1288) = (27.7/18.1) / ( (1/18.1)s + 1)
tau_used = 1/18.1288; 
t_99pct_up=-tau_used*log(1-0.99); t_99pct_down=tau_used*log(100);

fprintf('\n--- Các chỉ số đánh giá đáp ứng (Sử dụng v_ref = %.1f mm/s) ---\n',v_ref);
fprintf('Giá trị ổn định ước lượng (final_est) = %.2f mm/s (sử dụng = %.2f)\n',final_est,final_used);
fprintf('Peak = %.2f mm/s → Phần trăm Vọt lố (Overshoot) = %.2f %%\n',peak_value,PO);
fprintf('Thời gian tăng (10–90%%) = %.3f s\n',trise);
fprintf('Thời gian xác lập (±5%%) = %.3f s\n',t_settle);
fprintf('Thời gian đạt 99%% tốc độ cơ sở = %.3f s\n',t_to_base);
fprintf('Hằng số thời gian τ phân tích = %.3fs → Thời gian tăng 99%% = %.3fs, Thời gian giảm 99%% = %.3fs\n\n',...
        tau_used,t_99pct_down,t_99pct_down); % Sửa lỗi copy-paste ở đây