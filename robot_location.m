x = inputdlg('Enter step time to visualize',... %Introducing the snapshot to visualize
			'Input', [1 20]);

index = str2double(x{:})
Robot = [0 -0.2 0 1;0.4 0 0 1;0 0.2 0 1]'; % The Robot icon is a triangle
% tita = 90;
tita = pi/2;
xt = 0;
yt = 0;


for index=2:523
    % calculo de odometria
    R = data_enc(index, 7) - data_enc(index-1, 7);
    L = data_enc(index, 6) - data_enc(index-1, 6);
    Sc = (( R + L)/2)/1000; % (Right + Left) / 2 y lo pasamos a metros
    titac = ( R - L )/(width); % (Right - Left) / 2*S
    
    xt = xt + Sc*cos(tita);
    yt = yt + Sc*sin(tita);
    
    tita =  mod(titac + tita,2*pi); 
    
    traject_calc(index-1,:) = [xt, yt, tita];
    
    % FIN calculo odometria
end

for index=1:522 % Use the for loop to see a movie
	t = 0: 2*pi/359 : 2*pi;
    subplot(1,2,1)
	P = polarplot(t, 4.5 * ones(size(t)));% to fix the limits

	set(P, 'Visible', 'off')
	polarplot(t, lds_dis (index,2:361), '--g'); % Ploting the laser data wrt Robot frame
	title ('Laser data at Robot Reference Frame','FontWeight','bold','FontSize',16)
	subplot(1,2,2)
	title ('Data on Wordl Reference Frame', 'FontWeight','bold','FontSize',16)
	axis([-3 3 -2 4])
	grid on
	hold on

	for i=1:4 % plotting the 4 Land Marks
		circle (LandMark(i,:)',0.15)
	end


    % Polar2cartesian
    
    for i=1:360
        xt(i) = (cosd(i) * lds_dis(index,i+1) )/1000; % y pasamos a metros
        yt(i) = (sind(i) * lds_dis(index,i+1) )/1000; % y pasamos a metros
        % robot to world
        tita_ini = traject_calc(index,3);
        pos_act_x = traject_calc(index,1);
        pos_act_y = traject_calc(index,2);
         tst1 = transl2(pos_act_x ,pos_act_y )*trot2(tita_ini)*[xt(i),yt(i),1]';
         tst1 = tst1';
         ldx_calc(index,i) = tst1(1,1);
         ldy_calc(index,i) = tst1(1,2);
         %end robot to world plot
    end
    % FIN Polar2cartesian
    
    % robot to world plot
	scatter(ldx_calc(index,:), ldy_calc(index,:), 'r') % plotting the land mark seen by the Robot wrt wordl reference frame calculated by us
    % end robot to world plot
    
    % ploting the calculated trajectory
    plot (trajec_calc(:,1), trajec_calc(:,2), 'b.','LineWidth',1.5) % Plotting the trajectory
    % end ploting the calculated trajectory
    
    % scatter(ldx(index,:), ldy(index,:),'b') % plotting the land mark seen by the Robot wrt wordl reference frame
	% plot (trajec(:,1), trajec(:,2), 'r.','LineWidth',1.5) % Plotting the trajectory
	Robot_tr = transl(trajec(index,1),trajec(index,2),0)*trotz(mod(trajec(index,3)+pi/2,2*pi))*Robot; % moving the robot
	patch(Robot_tr(1,:), Robot_tr(2,:),'b');
    plot_ellipse(pk.signals.values(1:2,1:2,index),[trajec(index,1), trajec(index,2)],'g'); % Plotting the covariance matrix
	pause(0.1);
	clf
end

%Apartado 4: filtering landmarks
for k=1:length(lds_dis)
    array = lds_dis(k,2:361);
    edgeArray = diff([0; (array(:) ~= 0); 0]);
    indices = [find(edgeArray > 0)-1 find(edgeArray < 0)];
    j=1;
    i=1;
    while i<=length(indices)
        ini = indices(i,1)+2;
        while (i+1 <= length(indices)) && (indices(i,2)==indices(i+1,1) || indices(i,2)+1==indices(i+1,1))
            i = i + 1;
        end
        fin = indices(i,2);
        r = median(nonzeros(lds_dis(k,ini:fin)));
        a = (ini+fin)/2;
        x = r * cosd(a)/1000;
        y =  r * sind(a)/1000;
        %landmarks{i}{j}(1:2) contiene las coordenadas del landmark
        %detectadas por el laser
        landmarks{k}{j} = [x y];
        j = j + 1;
        i = i + 1;
    end
end

%Apartado 4: Finding associated landmark and printing it as seen by
%the robot
for t=1:523
    for l=1:length(landmarks{t})
        hold on
        scatter(landmarks{t}{l}(1),landmarks{t}{l}(2))
        distances = sqrt(sum(bsxfun(@minus, LandMark, landmarks{t}{l}).^2,2));
        %landmarks{i}{j}(3:4) contiene el landmark asociado a las
        %coordenadas de landmarks{i}{j}(1:2)
        landmarks{t}{l}(3:4) = LandMark(find(distances==min(distances),1),:);
    end
end

%Apartado 5: Similarity	Transform, error calculation
Error = [];
for t=1:523
    LandMark = [];
    detected = [];
    for m=1:length(landmarks{t})
        LandMark = [ LandMark'; landmarks{t}{m}(3:4)]';
        detected = [detected'; landmarks{t}{m}(1:2)]';
    end

    %Build Matrix A
    A = [];
    for i=1:size( LandMark , 2)
     A = [A;[ LandMark(1,i), LandMark(2,i),1,0]];
     A = [A;[ LandMark(2,i),-LandMark(1,i),0,1]];
    end
    B = [];%Build Matrix B
    for i=1:size( detected , 2)
     B = [B; detected(1,i); detected(2,i)];
    end
    %Compute tx ty i tita
    X = inv((A'*A))*A'*B;
    Error(t,1) = X(3);
    Error(t,2)= X(4);
    Error(t,3) = atan2(X(2),X(1))*180/pi;
end
