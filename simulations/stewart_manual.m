
run('stewart_setup.m');
config = load_config();
if isfield(config, 'manual_sens_step'), sens_step = config.manual_sens_step; else, sens_step = 0.2; end
if isfield(config, 'manual_sens_init'), sens_init = config.manual_sens_init; else, sens_init = 1.0; end

ball_x  = 0;  ball_y  = 0;
ball_vx = 0;  ball_vy = 0;
ball_state   = 0;
fall_pos     = zeros(3,1);
fall_vel     = zeros(3,1);
t_fall_start = 0;
t_floor_hit  = -inf;
RESET_DELAY  = 1.5;    % seconds on floor before auto-reset

pitch_cmd = 0;  roll_cmd = 0;
heave_cmd = h0;             % Current platform height [m] (fixed)
heave_vel = 0;              % Platform vertical velocity [m/s]
max_tilt  = 30 * deg2rad;
rate_fast = 20 * deg2rad;   % Arrow keys [rad/s]
rate_fine =  3 * deg2rad;   % A/D fine roll [rad/s]
t_survived = 0;

fig = figure('Name', 'Stewart Platform - Manual Control  [Arrows=tilt | W/S=sensitivity | A/D=roll | R=reset]', ...
    'NumberTitle','off','Color',[0.07 0.07 0.09], ...
    'Position',[80 40 1100 740]);
set(fig,'Renderer','opengl','RendererMode','manual');

fig.UserData = struct( ...
    'up',false,'down',false,'left',false,'right',false, ...
    'w',false,'a',false,'s',false,'d',false,'reset',false, ...
    'sens_mult', sens_init, 'sens_step', sens_step);
set(fig,'KeyPressFcn',  @kp_cb);
set(fig,'KeyReleaseFcn',@kr_cb);

ax = axes('Parent',fig,'Color',[0.05 0.05 0.07], ...
    'XColor',[0.55 0.55 0.55],'YColor',[0.55 0.55 0.55],'ZColor',[0.55 0.55 0.55], ...
    'GridColor',[0.28 0.28 0.28],'GridAlpha',0.4);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,90,30);

lim = R_base * 1.6;
xlim(ax,[-lim lim]); ylim(ax,[-lim lim]); zlim(ax,[0 h0*2.6]);
axis(ax,'manual');
xlabel(ax,'X [m]','Color',[0.6 0.6 0.6]);
ylabel(ax,'Y [m]','Color',[0.6 0.6 0.6]);
zlabel(ax,'Z [m]','Color',[0.6 0.6 0.6]);
title(ax,'MANUAL CONTROL  |  Arrows = tilt  |  W/S = sensitivity  |  A/D = fine roll  |  R = reset', ...
    'Color',[0.92 0.92 0.92],'FontSize',11,'FontWeight','bold');

n_ring  = 20;
theta_c = linspace(0,2*pi,n_ring)';
cos_tc  = cos(theta_c);
sin_tc  = sin(theta_c);

fill3(ax,R_base*cos_tc',R_base*sin_tc',zeros(1,n_ring), ...
    [0.28 0.28 0.38],'FaceAlpha',0.55,'EdgeColor',[0.5 0.5 0.65],'LineWidth',1.5);
plot3(ax,P_base(:,1),P_base(:,2),P_base(:,3),'o', ...
    'Color',[0.8 0.8 0.8],'MarkerSize',6,'MarkerFaceColor',[0.45 0.45 0.55]);

plot3(ax,0,0,h0+0.001,'+','Color',[0.2 1.0 0.3],'MarkerSize',20,'LineWidth',2.5);
plot3(ax,0,0,h0+0.001,'o','Color',[0.2 1.0 0.3],'MarkerSize',8,'LineWidth',1.5);

ring_w0 = (eye(3)*(R_top*[cos_tc,sin_tc,zeros(n_ring,1)]') + [0;0;h0])';
h_top   = fill3(ax,ring_w0(:,1)',ring_w0(:,2)',ring_w0(:,3)', ...
    [0.18 0.42 0.82],'FaceAlpha',0.80,'EdgeColor',[0.38 0.62 1.0],'LineWidth',2);
Pt0     = P_top_b + [0 0 h0];
h_top_pts = plot3(ax,Pt0(:,1),Pt0(:,2),Pt0(:,3),'o', ...
    'Color',[0.9 0.9 0.9],'MarkerSize',7,'MarkerFaceColor',[0.28 0.52 0.92]);

leg_colors = [0.30,0.60,1.00; 1.00,0.40,0.30; 0.30,0.90,0.40;
              1.00,0.80,0.10; 0.80,0.30,1.00; 0.20,0.90,0.90];
leg_xm = nan(18,1); leg_ym = nan(18,1); leg_zm = nan(18,1);
for k = 1:6
    idx = (k-1)*3+1;
    tp = P_top_b(k,:) + [0 0 h0];
    leg_xm(idx)=P_base(k,1); leg_xm(idx+1)=tp(1);
    leg_ym(idx)=P_base(k,2); leg_ym(idx+1)=tp(2);
    leg_zm(idx)=P_base(k,3); leg_zm(idx+1)=tp(3);
end
h_legs = gobjects(6,1);
for k = 1:6
    idx = (k-1)*3+1;
    h_legs(k) = plot3(ax,leg_xm(idx:idx+1),leg_ym(idx:idx+1),leg_zm(idx:idx+1),'-','Color',leg_colors(k,:),'LineWidth',2.8);
end
h_ctr = plot3(ax,0,0,h0,'o','Color',[1 0.8 0.2],'MarkerSize',7,'MarkerFaceColor',[1 0.8 0.2]);

[sp_x,sp_y,sp_z] = sphere(6);
ball_surf = surf(ax, sp_x*r_ball, sp_y*r_ball, sp_z*r_ball+h0+r_ball, ...
    'FaceColor',[0.84 0.84 0.92],'EdgeColor','none','FaceLighting','none');

h_shadow = plot3(ax,0,0,0.001,'o','Color',[0.18 0.18 0.24], ...
    'MarkerSize',7,'MarkerFaceColor',[0.16 0.16 0.20]);

trail_len = 12;
trail_buf = zeros(trail_len,3);
h_trail   = plot3(ax,trail_buf(:,1),trail_buf(:,2),trail_buf(:,3), ...
    '-','Color',[0.3 0.9 1.0 0.65],'LineWidth',1.5);

hx = -lim*0.90;  hy = -lim*0.90;
t_hud1  = text(ax,hx,hy,h0*2.45,'Pitch: +0.0   Roll: +0.0 deg', ...
    'Color',[0.92 0.88 0.20],'FontSize',10,'FontWeight','bold');
t_hud2  = text(ax,hx,hy,h0*2.28,'Ball: (0.0, 0.0) cm', ...
    'Color',[0.55 0.92 0.55],'FontSize',9);
t_hud3  = text(ax,hx,hy,h0*2.13,'Survived: 0.00 s', ...
    'Color',[1 0.8 0.2],'FontSize',11,'FontWeight','bold');


t_status= text(ax,0,0,h0*2.45,'GAME ON', ...
    'Color',[0.2 1.0 0.3],'FontSize',14,'FontWeight','bold','HorizontalAlignment','center');

dt           = 0.010;
RENDER_EVERY = 3;
step         = 0;
tic;
fprintf('Manual control running. Close window to exit.\n\n');

while ishandle(fig)

    try
        ud = fig.UserData;
    catch
        break;
    end

    if ud.reset
        ball_x=0; ball_y=0; ball_vx=0; ball_vy=0;
        pitch_cmd=0; roll_cmd=0; heave_cmd=h0; heave_vel=0;
        t_survived=0; ball_state=0;
        ud.reset = false;
        fig.UserData = ud;
    end

    df = rate_fast * ud.sens_mult * dt;
    if ud.up,    pitch_cmd = pitch_cmd - df; end
    if ud.down,  pitch_cmd = pitch_cmd + df; end
    if ud.left,  roll_cmd  = roll_cmd  + df; end   % Swapped direction
    if ud.right, roll_cmd  = roll_cmd  - df; end   % Swapped direction

    heave_prev = heave_cmd;
    heave_vel_prev = heave_vel;
    heave_vel = 0;

    ds = rate_fine * dt;
    if ud.a, roll_cmd  = roll_cmd  + ds; end
    if ud.d, roll_cmd  = roll_cmd  - ds; end

    pitch_cmd = max(-max_tilt, min(max_tilt, pitch_cmd));
    roll_cmd  = max(-max_tilt, min(max_tilt, roll_cmd));

    Rx = [1,0,0; 0,cos(roll_cmd),-sin(roll_cmd); 0,sin(roll_cmd),cos(roll_cmd)];
    Ry = [cos(pitch_cmd),0,sin(pitch_cmd); 0,1,0; -sin(pitch_cmd),0,cos(pitch_cmd)];
    R  = Ry*Rx;

    now = toc;

    if ball_state == 0   %--- ON PLATE ---
        t_survived = t_survived + dt;
        abx = (5/7)*g_acc*sin(pitch_cmd) - c_roll*ball_vx;
        aby = -(5/7)*g_acc*sin(roll_cmd)  - c_roll*ball_vy;
        ball_vx = ball_vx + abx*dt;
        ball_vy = ball_vy + aby*dt;
        ball_x  = ball_x  + ball_vx*dt;
        ball_y  = ball_y  + ball_vy*dt;

        if norm([ball_x,ball_y]) > r_limit
            fall_pos     = R*[ball_x;ball_y;r_ball] + [0;0;heave_cmd];
            fall_vel     = R*[ball_vx;ball_vy;0] + [0;0;heave_vel];
            t_fall_start = now;
            ball_state   = 1;
        end

        heave_acc = (heave_vel - heave_vel_prev) / dt;
        if heave_acc < -g_acc
            fall_pos     = R*[ball_x;ball_y;r_ball] + [0;0;heave_cmd];
            fall_vel     = R*[ball_vx;ball_vy;0] + [0;0;heave_vel_prev];
            t_fall_start = now;
            ball_state   = 1;
        end

    elseif ball_state == 1   %--- FREE FALL ---
        t_fall = now - t_fall_start;
        bw_fall = fall_pos + fall_vel*t_fall + [0;0;-0.5*g_acc*t_fall^2];

        plate_z = heave_cmd;   % plate surface world-z (approx, ignoring tilt at center)
        ball_bottom_z = bw_fall(3) - r_ball;
        ball_vel_now = fall_vel + [0;0;-g_acc*t_fall];

        if ball_bottom_z <= plate_z && ball_vel_now(3) <= heave_vel
            bp = R' * (bw_fall - [0;0;heave_cmd]);
            if norm(bp(1:2)) < r_limit
                ball_x  = bp(1);
                ball_y  = bp(2);
                bv_plate = R' * (ball_vel_now - [0;0;heave_vel]);
                ball_vx = bv_plate(1);
                ball_vy = bv_plate(2);
                ball_state = 0;
            end
        end

        if ball_state == 1 && bw_fall(3) <= r_ball
            ball_state  = 2;
            t_floor_hit = now;
        end

    elseif ball_state == 2   %--- ON FLOOR, WAITING ---
        if now - t_floor_hit > RESET_DELAY
            ball_x=0; ball_y=0; ball_vx=0; ball_vy=0;
            heave_cmd=h0; heave_vel=0; heave_vel_prev=0;
            t_survived=0; ball_state=0;
        end
    end

    step = step + 1;
    if mod(step, RENDER_EVERY) ~= 0, continue; end
    if ~ishandle(fig), break; end

    rw = (R*(R_top*[cos_tc,sin_tc,zeros(n_ring,1)]') + [0;0;heave_cmd])';
    set(h_top,'XData',rw(:,1)','YData',rw(:,2)','ZData',rw(:,3)');

    Pt = (R*P_top_b' + [0;0;heave_cmd])';
    set(h_top_pts,'XData',Pt(:,1),'YData',Pt(:,2),'ZData',Pt(:,3));
    for k = 1:6
        idx=(k-1)*3+1;
        leg_xm(idx+1)=Pt(k,1); leg_ym(idx+1)=Pt(k,2); leg_zm(idx+1)=Pt(k,3);
        set(h_legs(k),'XData',leg_xm(idx:idx+1),'YData',leg_ym(idx:idx+1),'ZData',leg_zm(idx:idx+1));
    end
    set(h_ctr,'ZData',heave_cmd);

    if ball_state == 0
        bw = R*[ball_x;ball_y;r_ball] + [0;0;heave_cmd];
        ball_col = [0.84 0.84 0.92];
        tc = [0.3,0.9,1.0,0.65];
        set(h_shadow,'XData',ball_x,'YData',ball_y,'ZData',0.001,'Visible','on');
        trail_buf(1:end-1,:) = trail_buf(2:end,:);
        trail_buf(end,:) = bw';
    elseif ball_state == 1
        t_fall = now - t_fall_start;
        bw = fall_pos + fall_vel*t_fall + [0;0;-0.5*g_acc*t_fall^2];
        bw = max(bw, [bw(1);bw(2);r_ball]);   % floor clamp
        ball_col = [1.0 0.35 0.18];
        tc = [1.0,0.35,0.18,0.60];
        set(h_shadow,'Visible','off');
        trail_buf(1:end-1,:) = trail_buf(2:end,:);
        trail_buf(end,:) = bw';
    else   % on floor
        bw = [fall_pos(1); fall_pos(2); r_ball];
        t_fall_end = t_floor_hit - t_fall_start;
        bw = fall_pos + fall_vel*t_fall_end + [0;0;-0.5*g_acc*t_fall_end^2];
        bw(3) = r_ball;
        ball_col = [1.0 0.25 0.12];
        tc = [1.0,0.25,0.12,0.50];
        set(h_shadow,'Visible','off');
    end

    set(ball_surf,'XData',bw(1)+sp_x*r_ball,'YData',bw(2)+sp_y*r_ball, ...
        'ZData',bw(3)+sp_z*r_ball,'FaceColor',ball_col);
    set(h_trail,'XData',trail_buf(:,1),'YData',trail_buf(:,2), ...
        'ZData',trail_buf(:,3),'Color',tc);

    set(t_hud1,'String',sprintf('Pitch: %+.1f   Roll: %+.1f   Sens: %.1fx', ...
        pitch_cmd/deg2rad, roll_cmd/deg2rad, ud.sens_mult));
    set(t_hud2,'String',sprintf('Ball: (%.1f, %.1f) cm', ball_x*100, ball_y*100));
    set(t_hud3,'String',sprintf('Survived: %.2f s', t_survived));

    if ball_state == 0
        set(t_status,'String','','Color',[0.2 1.0 0.3]);
    elseif ball_state == 1
        set(t_status,'String','BALL FALLING!','Color',[1.0 0.55 0.1]);
    else
        set(t_status,'String','BALL LOST!  Resetting...','Color',[1.0 0.25 0.18]);
    end

    drawnow limitrate;

    slack = step*dt - toc;
    if slack > 0.040
        pause(slack - 0.015);
    end
end

fprintf('Manual control session ended.\n');

function kp_cb(src, evt)
    ud = src.UserData;
    switch evt.Key
        case 'uparrow',    ud.up    = true;
        case 'downarrow',  ud.down  = true;
        case 'leftarrow',  ud.left  = true;
        case 'rightarrow', ud.right = true;
        case 'w'
            if ~ud.w
                ud.sens_mult = ud.sens_mult + ud.sens_step;
                ud.w = true;
            end
        case 's'
            if ~ud.s
                ud.sens_mult = max(0.1, ud.sens_mult - ud.sens_step);
                ud.s = true;
            end
        case 'a',          ud.a     = true;
        case 'd',          ud.d     = true;
        case 'r',          ud.reset = true;
    end
    src.UserData = ud;
end

function kr_cb(src, evt)
    ud = src.UserData;
    switch evt.Key
        case 'uparrow',    ud.up    = false;
        case 'downarrow',  ud.down  = false;
        case 'leftarrow',  ud.left  = false;
        case 'rightarrow', ud.right = false;
        case 'w',          ud.w     = false;
        case 's',          ud.s     = false;
        case 'a',          ud.a     = false;
        case 'd',          ud.d     = false;
    end
    src.UserData = ud;
end
