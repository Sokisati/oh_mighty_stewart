function visualize_stewart(sim_result, in_Kp, in_Ki, in_Kd, disturb_table)
    if nargin < 5
        disturb_table = [];
    end

    run('stewart_setup.m');
    
    g_acc = 9.81; % fallback
    
    ball_x_pid  = sim_result.ball_x;
    ball_y_pid  = sim_result.ball_y;
    ball_vx_pid = sim_result.ball_vx;
    ball_vy_pid = sim_result.ball_vy;
    pitch_log   = sim_result.pitch_cmd;
    roll_log    = sim_result.roll_cmd;
    N           = sim_result.N;
    t_vec       = sim_result.t_vec;
    
    if N < 2
        fprintf('Not enough data to visualize.\n');
        return;
    end
    
    pos_ref_pid = zeros(N, 6);
    pos_ref_pid(:, 3) = h0;
    pos_ref_pid(2:end, 4) = roll_log(1:end-1);
    pos_ref_pid(2:end, 5) = pitch_log(1:end-1);
    
    fprintf('Pre-computing animation data...\n');
    theta_c  = linspace(0, 2*pi, 20)';
    n_ring   = numel(theta_c);
    cos_tc   = cos(theta_c); sin_tc = sin(theta_c);

    ball_world_pid  = zeros(N, 3);
    ball_on_plate_pid = true(N, 1);
    top_ring_pid    = zeros(n_ring,3,N);
    P_top_pid       = zeros(6,3,N);
    leg_x_pid       = nan(18,N);
    leg_y_pid       = nan(18,N);
    leg_z_pid       = nan(18,N);

    fall_start_pid  = N;
    fall_pos_pid    = zeros(3,1);
    fall_vel_pid    = zeros(3,1);

    for i = 1:N
        rx_i=pos_ref_pid(i,4); ry_i=pos_ref_pid(i,5); rz_i=pos_ref_pid(i,6);
        Rx_i=[1,0,0;0,cos(rx_i),-sin(rx_i);0,sin(rx_i),cos(rx_i)];
        Ry_i=[cos(ry_i),0,sin(ry_i);0,1,0;-sin(ry_i),0,cos(ry_i)];
        Rz_i=[cos(rz_i),-sin(rz_i),0;sin(rz_i),cos(rz_i),0;0,0,1];
        R_i = Rz_i*Ry_i*Rx_i;

        ring_local = R_top * [cos_tc, sin_tc, zeros(n_ring,1)]';
        ring_world = R_i * ring_local;
        top_ring_pid(:,:,i) = (ring_world + [0;0;h0])';
        for k = 1:6
            P_top_pid(k,:,i) = (R_i*P_top_b(k,:)' + [0;0;h0])';
            idx = (k-1)*3 + 1;
            leg_x_pid(idx,  i) = P_base(k,1);  leg_x_pid(idx+1,i) = P_top_pid(k,1,i);
            leg_y_pid(idx,  i) = P_base(k,2);  leg_y_pid(idx+1,i) = P_top_pid(k,2,i);
            leg_z_pid(idx,  i) = P_base(k,3);  leg_z_pid(idx+1,i) = P_top_pid(k,3,i);
        end

        if ball_on_plate_pid(i)
            bw = R_i*[ball_x_pid(i);ball_y_pid(i);r_ball] + [0;0;h0];
            ball_world_pid(i,:) = bw';
            if norm([ball_x_pid(i), ball_y_pid(i)]) > r_limit && i < N
                ball_on_plate_pid(i+1:end) = false;
                fall_start_pid = i + 1;
                fall_pos_pid   = bw;
                fall_vel_pid   = R_i * [ball_vx_pid(i); ball_vy_pid(i); 0];
            end
        else
            t_fall = t_vec(i) - t_vec(fall_start_pid);
            bw = fall_pos_pid + fall_vel_pid*t_fall + [0;0;-0.5*g_acc*t_fall^2];
            if bw(3) < r_ball
                bw(3) = r_ball;
                ball_world_pid(i:end,:) = repmat(bw', N-i+1, 1);
                break;
            end
            ball_world_pid(i,:) = bw';
        end
    end

    fig = figure('Name', 'Stewart Platform - 3D Simulation', ...
                 'NumberTitle', 'off', 'Color', [0.10 0.10 0.13], ...
                 'Position', [100, 50, 1000, 700]);
    set(fig, 'Renderer', 'opengl', 'RendererMode', 'manual');

    ax = axes('Parent', fig, 'Color', [0.08 0.08 0.10], ...
              'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
              'ZColor', [0.7 0.7 0.7], 'GridColor', [0.35 0.35 0.35], ...
              'GridAlpha', 0.4);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    view(ax, 40, 28);

    lim = R_base * 1.5;
    xlim(ax,[-lim lim]); ylim(ax,[-lim lim]); zlim(ax,[0 h0*2.4]);
    axis(ax, 'manual');
    xlabel(ax,'X [m]','Color',[0.8 0.8 0.8]);
    ylabel(ax,'Y [m]','Color',[0.8 0.8 0.8]);
    zlabel(ax,'Z [m]','Color',[0.8 0.8 0.8]);
    title(ax,'Stewart Platform 3D', 'Color',[0.95 0.95 0.95],'FontSize',13,'FontWeight','bold');

    time_txt  = text(ax,-lim*.9,-lim*.9,h0*2.2,'t = 0.00 s', 'Color',[0.9 0.9 0.2],'FontSize',10,'FontWeight','bold');
    mode_txt  = text(ax,-lim*.9,-lim*.9,h0*2.05,'SIMULATION', 'Color',[0.3 1.0 0.4],'FontSize',10,'FontWeight','bold');
    kp_txt = text(ax, lim*0.5, lim*0.8, h0*2.2, sprintf('Kp: %.3f', in_Kp), 'Color',[0.3 1.0 0.4],'FontSize',11,'FontWeight','bold');
    ki_txt = text(ax, lim*0.5, lim*0.8, h0*2.0, sprintf('Ki: %.3f', in_Ki), 'Color',[0.3 0.8 1.0],'FontSize',11,'FontWeight','bold');
    kd_txt = text(ax, lim*0.5, lim*0.8, h0*1.8, sprintf('Kd: %.3f', in_Kd), 'Color',[1.0 0.8 0.2],'FontSize',11,'FontWeight','bold');

    leg_colors = [0.30,0.60,1.00; 1.00,0.40,0.30; 0.30,0.90,0.40;
                  1.00,0.80,0.10; 0.80,0.30,1.00; 0.20,0.90,0.90];

    fill3(ax, R_base*cos_tc', R_base*sin_tc', zeros(1,n_ring), ...
          [0.35 0.35 0.45],'FaceAlpha',0.6,'EdgeColor',[0.6 0.6 0.7],'LineWidth',1.5);
    plot3(ax, P_base(:,1), P_base(:,2), P_base(:,3), 'o', ...
          'Color',[0.9 0.9 0.9],'MarkerSize',7,'MarkerFaceColor',[0.6 0.6 0.7]);
    plot3(ax, 0, 0, h0+0.001, '+', 'Color', [0.3 1.0 0.3], 'MarkerSize', 16, 'LineWidth', 2.5);

    tr0 = top_ring_pid(:,:,1);
    h_top = fill3(ax,tr0(:,1)',tr0(:,2)',tr0(:,3)', ...
                  [0.20,0.45,0.80],'FaceAlpha',0.75,'EdgeColor',[0.5 0.7 1.0],'LineWidth',2);
    h_top_pts = plot3(ax,P_top_pid(:,1,1),P_top_pid(:,2,1),P_top_pid(:,3,1), ...
                      'o','Color',[0.9 0.9 0.9],'MarkerSize',7,'MarkerFaceColor',[0.3 0.6 0.9]);

    h_legs = gobjects(6,1);
    for k = 1:6
        idx = (k-1)*3 + 1;
        h_legs(k) = plot3(ax, leg_x_pid(idx:idx+1,1), leg_y_pid(idx:idx+1,1), leg_z_pid(idx:idx+1,1), ...
            '-', 'Color', leg_colors(k,:), 'LineWidth', 2.8);
    end

    h_ctr = plot3(ax,0,0,h0,'o','Color',[1 0.8 0.2],'MarkerSize',8, 'MarkerFaceColor',[1 0.8 0.2],'LineWidth',1);

    [sp_x,sp_y,sp_z] = sphere(6);
    ball_surf = surf(ax, sp_x*r_ball, sp_y*r_ball, sp_z*r_ball+h0+r_ball, ...
        'FaceColor',[0.78 0.78 0.85],'EdgeColor','none','FaceLighting','none');

    trail_len = 15;
    trail_buf = repmat(ball_world_pid(1,:), trail_len, 1);
    h_trail   = plot3(ax,trail_buf(:,1),trail_buf(:,2),trail_buf(:,3), ...
        '-','Color',[0.3 0.9 1.0 0.7],'LineWidth',1.5);

    fprintf('Animation starting...  Close figure to stop.\n\n');
    RENDER_EVERY = 3; 
    tic;
    for i = 1:N
        if ~ishandle(fig), break; end
        if mod(i, RENDER_EVERY) ~= 0 && i ~= N, continue; end

        tr = top_ring_pid(:,:,i);
        set(h_top,     'XData',tr(:,1)',         'YData',tr(:,2)',         'ZData',tr(:,3)');
        set(h_top_pts, 'XData',P_top_pid(:,1,i),'YData',P_top_pid(:,2,i),'ZData',P_top_pid(:,3,i));
        for k = 1:6
            idx = (k-1)*3 + 1;
            set(h_legs(k), 'XData',leg_x_pid(idx:idx+1,i), 'YData',leg_y_pid(idx:idx+1,i), 'ZData',leg_z_pid(idx:idx+1,i));
        end

        tz = pos_ref_pid(i,3);
        set(h_ctr,'XData',0,'YData',0,'ZData',tz);

        bw = ball_world_pid(i,:);
        set(ball_surf,'XData',bw(1)+sp_x*r_ball,'YData',bw(2)+sp_y*r_ball,'ZData',bw(3)+sp_z*r_ball);
        trail_buf(1:end-1,:) = trail_buf(2:end,:);
        trail_buf(end,:) = bw;
        tc = [0.3, 0.9, 1.0, 0.7];                      
        if ~ball_on_plate_pid(i), tc = [1.0,0.15,0.15,0.9]; end  
        set(h_trail,'XData',trail_buf(:,1),'YData',trail_buf(:,2),'ZData',trail_buf(:,3),'Color',tc);

        set(time_txt,'String',sprintf('t = %.2f s', t_vec(i)));
        
        if isempty(disturb_table)
            is_disturb = false;
        else
            is_disturb = any(abs(t_vec(i) - disturb_table(:,1)) < 0.4);
        end
        
        if is_disturb
            set(mode_txt,'String','DISTURBANCE!','Color',[1.0 0.3 0.2]);
        else
            set(mode_txt,'String','SIMULATION','Color',[0.3 1.0 0.4]);
        end

        drawnow; 

        elapsed = toc;
        sim_time = t_vec(i);
        if elapsed < sim_time
            slack = sim_time - elapsed;
            if slack > 0.020
                pause(slack - 0.015);
            end
        end
    end
    fprintf('Animation complete.\n');
end
