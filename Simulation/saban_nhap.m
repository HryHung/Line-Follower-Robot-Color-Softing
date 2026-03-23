
clc; clear; close all;

%--- Các điểm
s = sqrt(2);
A = [3000, 0];
B = [500, 0];
C = [0, 500];
D = [500, 1000];
E = [1800 - 800*s, 1000];
F = [1800 - 400*s, 1800 - 400*s];
H = [1800, 1500];
K = [3000,1500];
S = [1000,0];
G = [1800 - 400*s, 200 + 400*s];
I = [1800, 500];
Lp = [3000, 500];   % dùng Lp để không trùng với lệnh

R_arc  = 800;
R_arc1 = 500;
trackWidth = 26;

%--- Phân đoạn track
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

% Gom điểm để vẽ label
points = struct('A',A,'B',B,'C',C,'D',D,'E',E,'F',F,...
                'H',H,'K',K,'S',S,'G',G,'I',I,'Lp',Lp);

%--- Vẽ: toàn bộ track màu đen
figure('Name','Line follower with branching (black track)'); hold on; axis equal; grid on;
drawTrackBlack([segments_common, segments_red, segments_green]);
drawPoints(points);
title('Track màu đen — xe sẽ rẽ nhánh tùy hàng hóa');

% Active segments ban đầu là common
activeSegments = segments_common;

%--- Xe
L = 150;   % khoảng cách giữa 2 bánh
W = 100;   % chiều rộng thân xe
H_robot = 200;   % chiều dài thân xe
v = 80;   % tốc độ tiến cơ bản

dt = 0.1;
T  = 1000;
N  = round(T/dt);

% Khởi tạo tại A
x = A(1);
y = A(2);
theta = atan2(B(2)-A(2), B(1)-A(1));

% PID
Kp = 20; Ki = 0.2; Kd = 0.2;
err_int = 0; prev_err = 0;

% 5 cảm biến
d_sensor = 220;
sensor_gap = 15;
weights = [-2 -1 0 1 2];

% Vẽ xe
hCar = patch(0,0,'r'); 
hWheelL = patch(0,0,'k'); 
hWheelR = patch(0,0,'k');
hSensors = plot(0,0,'bo','MarkerFaceColor','b');

% Vận tốc giới hạn
v_max = 300;
v_min = 50;

% Trạng thái
pickedCargo = false;
cargo = '';
branched = false;

% ------------------ Vết xe (A->C: blue, sau C: màu cargo) ------------------
passedS = false;
trailX_pre = []; trailY_pre = [];
trailX_post = []; trailY_post = [];
hTrailPre  = plot(NaN,NaN,'b-','LineWidth',1.5);    % pre-C: blue
hTrailPost = plot(NaN,NaN,'-','LineWidth',1.5);     % post-C: set color khi biết cargo
% ---------------------------------------------------------------------------

% cargoPlot handle (thùng hàng) sẽ tạo khi pick
cargoPlot = [];

for k = 1:N
    % Vị trí cảm biến
    [Xs,Ys] = sensorPositions(x,y,theta,d_sensor,sensor_gap);

    %--- Pick hàng tại S
    if ~pickedCargo && norm([x,y]-S) < 60
        cargo = input('Chọn màu hàng (red/green): ', 's'); % nhập chuỗi từ bàn phím
        while ~ismember(cargo, {'red','green'})
            cargo = input('Chỉ được nhập red hoặc green: ', 's');
        end
        pickedCargo = true;
        disp(['Cargo picked at S: ', cargo]);


        
        % tạo marker thùng hàng gắn vào xe (ban đầu trùng vị trí S)
        cargoPlot = plot(x, y, 's', 'MarkerSize',14, ...
                         'MarkerFaceColor', cargo, 'MarkerEdgeColor','k');
        pause(0.5)
        drawnow;
        
        % nếu đã qua S rồi thì set màu cho phần post
        if passedS
            set(hTrailPost, 'Color', cargo);
        end
    end

    %--- Trong vòng lặp cập nhật vị trí xe (nếu có cargo thì cập nhật vị trí marker)
    if ~isempty(cargoPlot)
        set(cargoPlot, 'XData', x, 'YData', y);
    end

    %--- Rẽ nhánh tại E
    if ~branched && pickedCargo && norm([x,y]-E) < 80
        if strcmp(cargo,'red')
            activeSegments = [segments_common segments_red];
        else
            activeSegments = [segments_common segments_green];
        end
        branched = true;
        disp(['Branched at E according to cargo = ', cargo]);
        
        % nếu đã biết cargo thì set màu cho trail post
        if pickedCargo
            set(hTrailPost, 'Color', cargo);
        end
    end
    
    % --- Kiểm tra điểm cuối (chỉ khi đã rẽ nhánh)
    if branched
        if strcmp(cargo,'red')
            goal = K;   % nhánh đỏ kết thúc ở K
        else
            goal = Lp;  % nhánh xanh kết thúc ở Lp
        end
        
        % kiểm tra sai số tới goal (±5 đơn vị)
        if norm([x,y]-goal) <= 18
            disp(['Reached destination at ', cargo, ' branch!']);
            % dừng mô phỏng (thoát for)
            break;
        end
    end

    % Đọc tín hiệu cảm biến
    signals = zeros(1,5);
    for i=1:5
        if onTrack(Xs(i),Ys(i),activeSegments)
            signals(i)=1;
        end
    end

    % Sai số
    if sum(signals)>0
        err = sum(weights.*signals)/sum(signals);
    else
        err = prev_err; % mất line thì giữ hướng cũ
    end
    fprintf('Vòng %d: Sai số err = %.2f\n', k, err);

    % PID
    err_int = err_int + err*dt;
    err_der = (err - prev_err)/dt;
    u = Kp*err + Ki*err_int + Kd*err_der;
    prev_err = err;

    % scale tốc độ
    scale = max(0.3, 1 - abs(err)/5); 
    v_dyn = v * scale;  

    % vận tốc bánh
    vl = v_dyn - u;
    vr = v_dyn + u;

    vl = min(max(vl, v_min), v_max);
    vr = min(max(vr, v_min), v_max);

    % kinematics
    v_rb = (vl+vr)/2;
    w = (vr-vl)/L;

    x = x + v_rb*cos(theta)*dt;
    y = y + v_rb*sin(theta)*dt;
    theta = theta + w*dt;

    x_err = x + v_rb*cos(theta)*dt + H_robot*cos(theta);
    y_err = y + v_rb*sin(theta)*dt + H_robot*sin(theta);

    % ---------------- cập nhật vết xe ----------------
    if ~passedS
        trailX_pre(end+1) = x_err;
        trailY_pre(end+1) = y_err;
    else
        trailX_post(end+1) = x_err;
        trailY_post(end+1) = y_err;
    end

    % kiểm tra qua điểm S (nếu chưa qua)
    if ~passedS && norm([x,y]-S) < 50   % ngưỡng 50: chỉnh nếu cần
        passedS = true;
        disp('Passed C: đổi vết sang màu cargo');
        % gán điểm nối để mượt (nếu có pre điểm cuối)
        if ~isempty(trailX_pre)
            trailX_post = [trailX_post, trailX_pre(end)];
            trailY_post = [trailY_post, trailY_pre(end)];
        end
        if pickedCargo
            set(hTrailPost, 'Color', cargo);
        end
    end

    % cập nhật hiển thị vết
    set(hTrailPre,  'XData', trailX_pre,  'YData', trailY_pre);
    set(hTrailPost, 'XData', trailX_post, 'YData', trailY_post);
    % --------------------------------------------------

    % vẽ xe
    [Xb,Yb] = transformRect(x, y, theta, W, H_robot);
    nx = -sin(theta); ny = cos(theta);
    [Xwl,Ywl] = transformRect(x + nx*L/2, y + ny*L/2, theta, 50, 20);
    [Xwr,Ywr] = transformRect(x - nx*L/2, y - ny*L/2, theta, 50, 20);

    set(hCar,   'XData',Xb,  'YData',Yb);
    set(hWheelL,'XData',Xwl, 'YData',Ywl);
    set(hWheelR,'XData',Xwr, 'YData',Ywr);
    set(hSensors,'XData',Xs,'YData',Ys);

    drawnow;
end



%% ================== HÀM VẼ & HỖ TRỢ ==================
