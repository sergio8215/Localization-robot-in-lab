x = inputdlg('Enter step time to visualize',... %Introducing the snapshot to visualize
			'Input', [1 20]);

index = str2double(x{:})
Robot = [0 -0.2 0 1;0.4 0 0 1;0 0.2 0 1]'; % The Robot icon is a triangle

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

	scatter(ldx(index,:), ldy(index,:)) % plotting the land mark seen by the Robot wrt wordl reference frame
    
    count_landm = 0;
    count_zero = 0;
    acum_x = 0;
    acum_y = 0;
    landmark_robot = [0,0];
    landm=2;
    
    if index == 201
        stop = 0;
    end
    
    while landm < 362
        landm_ini = landm;
        
        if lds_dis(index,landm) ~= 0
            while count_zero < 3 && landm < 362 % Si tenemos menos de 5 ceros seguidos estamos viendo un objeto
                if lds_dis(index,landm) == 0 % Verficamos que seguimos viendo un objeto
                    count_zero = count_zero+1;
                else
                    count_zero = 0;
                    acum_x = ldx(index,landm-1) + acum_x;
                    acum_y = ldy(index,landm-1) + acum_y;
                    count_landm = count_landm +1; % contamos las casillas donde vemos el objeto
                end
                landm =landm+1;
            end
                count_zero = 0;
                if count_landm > 3 % Si tenemos se a visto el landmark más de 3 veces, podemos decir que no es un error
                    media_x = acum_x/count_landm;
                    media_y = acum_y/count_landm;
                    landmark_robot = [landmark_robot ; media_x, media_y];
                 end
                count_landm = 0;
                acum_x = 0;
                acum_y = 0;
        else
            landm =landm+1;
        end
    end
    
	for i=2:size(landmark_robot) % plotting the 4 Land Marks
		circle (landmark_robot(i,:),0.15)
	end
    
    scatter(ldx(index,:), ldy(index,:)) % plotting the land mark seen by the Robot wrt wordl reference frame
	plot (trajec(:,1), trajec(:,2), 'r.','LineWidth',1.5) % Plotting the trajectory
	Robot_tr = transl(trajec(index,1),trajec(index,2),0)*trotz(mod(trajec(index,3)+pi/2,2*pi))*Robot; % moving the robot
	patch(Robot_tr(1,:), Robot_tr(2,:),'b');
    hold on
    for ellipses=1:10:index
        plot_ellipse(pk.signals.values(1:2,1:2,ellipses),[trajec(ellipses,1), trajec(ellipses,2)],'g'); % Plotting the covariance matrix
    end
	
	pause(0.1);
	clf
end