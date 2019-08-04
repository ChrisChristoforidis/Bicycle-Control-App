classdef simulation_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        BicycleRiderSimulationUIFigure  matlab.ui.Figure
        UIAxes                          matlab.ui.control.UIAxes
        PauseButton                     matlab.ui.control.Button
        ImpulseButton                   matlab.ui.control.Button
        CameraViewDropDownLabel         matlab.ui.control.Label
        CameraViewDropDown              matlab.ui.control.DropDown
        roll                            matlab.ui.control.UIAxes
        torque                          matlab.ui.control.UIAxes
        steer                           matlab.ui.control.UIAxes
        SignalsSwitchLabel              matlab.ui.control.Label
        SignalsSwitch                   matlab.ui.control.Switch
        MagnitudeNEditFieldLabel        matlab.ui.control.Label
        MagnitudeNEditField             matlab.ui.control.NumericEditField
        DisturbanceLabel                matlab.ui.control.Label
        DurationmsEditFieldLabel        matlab.ui.control.Label
        DurationmsEditField             matlab.ui.control.NumericEditField
        ForwardSpeedkmhDropDownLabel    matlab.ui.control.Label
        ForwardSpeedkmhDropDown         matlab.ui.control.DropDown
        UITable                         matlab.ui.control.Table
        ModelParametersLabel            matlab.ui.control.Label
        IntegrationTimeStepSpinnerLabel  matlab.ui.control.Label
        IntegrationTimeStepSpinner      matlab.ui.control.Spinner
        Switch                          matlab.ui.control.Switch
        BicycleAnimationLabel           matlab.ui.control.Label
        SteerAxisTiltdegSliderLabel     matlab.ui.control.Label
        SteerAxisTiltdegSlider          matlab.ui.control.Slider
        WhiteNoiseSwitchLabel           matlab.ui.control.Label
        WhiteNoiseSwitch                matlab.ui.control.Switch
        AzimuthSliderLabel              matlab.ui.control.Label
        AzimuthSlider                   matlab.ui.control.Slider
        ElevationSliderLabel            matlab.ui.control.Label
        ElevationSlider                 matlab.ui.control.Slider
        WheelRadiuscmEditFieldLabel     matlab.ui.control.Label
        WheelRadiuscmEditField          matlab.ui.control.NumericEditField
        WheelbasecmEditFieldLabel       matlab.ui.control.Label
        WheelbasecmEditField            matlab.ui.control.NumericEditField
        ControllerSwitchLabel           matlab.ui.control.Label
        ControllerSwitch                matlab.ui.control.Switch
        ResetButton                     matlab.ui.control.Button
    end

    properties (Access = public)
        fork_length=0.75;  % length from front wheel center to the midpoint of the handlebars
        trail_angle;       % steer axis tilt
        wheelbase;         % distance between wheel centers
        r;                 % radius of wheels
        delta=0;           % steer angle
        psi=0;             % yaw angle
        phi=0;             % roll angle
        xp; yp; zp;        % cartesian coordinates of the back wheel contact point  
        e3=[0;0;1];        % z - unit vector
        e2=[0;1;0];        % y - unit vector
        e1=[1;0;0];        % x - unit vector
        theta;             % angle for ploting circle points
        v;                 % forward velocity
        Gp;                % eom of bicycle with neuromuscular dynamics
        K;                 % controller gains
        freeze=0;          % pause flag 
        dt;                % time step
        noise_flag;        % white noise activation flag;
        duration;          % duration of impulseperturbation;
        step;              % counter for perturbation duration
        w=0;               % variable for perturbation magnitude
        magnitude;         % impulse max value / white noise variance
        camera='1';        % camera state variable
        state;             % state after integration
        t;                 % time 
        subplot_flag;      % flag for signals  display status
        bike_plot_flag;    % flag for animation activation status
        Trider=0;          % Rider torque
        torque_line;steer_line;roll_line;path_line; % animated lines for signal visualization
        p;rods;            % plot variables for bike segments
        omegac;            % resonance frequency of neuromuscular dynamics
        azimuth=60;elevation=30; % camera angles
    end
    methods (Access = private)
            
        function plotBike(app,segments)
        
              app.p(1)=plot3(app.UIAxes,segments.s1(1,:),segments.s1(2,:),segments.s1(3,:),'b','LineWidth',3);
              hold(app.UIAxes,'on');
              box(app.UIAxes,'on');
              app.p(2)=patch(app.UIAxes,segments.c1(1,:),segments.c1(2,:),segments.c1(3,:),'cyan','LineWidth',2);
              app.p(3)=patch(app.UIAxes,segments.c2(1,:),segments.c2(2,:),segments.c2(3,:),'cyan','LineWidth',2);
              app.p(4)=plot3(app.UIAxes,segments.s2(1,:),segments.s2(2,:),segments.s2(3,:),'b','LineWidth',3);
              app.p(5)=plot3(app.UIAxes,segments.s3(1,:),segments.s3(2,:),segments.s3(3,:),'b','LineWidth',3);
              app.p(6)=plot3(app.UIAxes,segments.s4(1,:),segments.s4(2,:),segments.s4(3,:),'b','LineWidth',3);
              for jj=10:10:60
                app.p(jj/10+8)=plot3(app.UIAxes,[segments.s7r(1,jj/10) segments.c1(1,jj)],[segments.s7r(2,jj/10) segments.c1(2,jj)],[segments.s7r(3,jj/10) segments.c1(3,jj)],'b','LineWidth',2);
                app.rods(jj/10)=plot3(app.UIAxes,[segments.s7f(1,jj/10) segments.c2(1,jj)],[segments.s7f(2,jj/10) segments.c2(2,jj)],[segments.s7f(3,jj/10) segments.c2(3,jj)],'b','LineWidth',2);
              end
              %addpoints(app.path_line,app.xp,app.yp,0)
              app.p(7)=patch(app.UIAxes,segments.s5(1,:),segments.s5(2,:),segments.s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue');
              app.p(8)=patch(app.UIAxes,segments.s6(1,:),segments.s6(2,:),segments.s6(3,:),'r','LineWidth',2.5,'EdgeColor','red');
              if (app.camera=='1')
                view(app.UIAxes,app.azimuth,app.elevation)
              elseif(app.camera=='2')
                 view(app.UIAxes,-90,90-app.trail_angle*180/pi)
              elseif (app.camera=='3')
                 view(app.UIAxes,-90,0)
  
              end
              ylim(app.UIAxes,[app.yp-1.2 app.yp+0.7])
              xlim(app.UIAxes,[app.xp-1 app.xp+2.5])
              zlim(app.UIAxes,[0 1.3])           
              app.UIAxes.XGrid='on';
              app.UIAxes.YGrid='on';
              app.UIAxes.ZLabel.String='Z';
              daspect(app.UIAxes,[1 1 1])
              %axis(app.UIAxes, 'equal');
              hold(app.UIAxes,'off');
        end
        
        function segments = updateSegments(app,t,nw)
              wb=app.wheelbase;%m
              lamda=app.trail_angle;%rad
              CD=app.fork_length;
              radr=app.r; radf=app.r;  %m
              thr=t*app.v/radr;
              r1=rotmat(app,-app.e3,app.psi);r2=rotmat(app,r1*app.e1,app.phi);rrf=r2*r1;
              r3=rotmat(app,app.e2,-lamda);
              r4=rotmat(app,r3*(-app.e3),app.delta);
              q2=r4*r3;
              q=rrf;
              rrw=rotmat(app,app.e2,thr);
              x=app.xp;
              y=app.yp;
              z=0;
              xr=radr*cos(app.theta);zr=radr*sin(app.theta)+radr; yr=0*app.theta;
              xf=radf*cos(app.theta)+wb;zf=radf*sin(app.theta)+radf;yf=0*app.theta;
              c1=rrw*[xr;yr;zr-radr];
              c1=q*[c1(1,:) ;c1(2,:); c1(3,:)+radr]; c2m=rotmat(app,q2*app.e2,thr)*q2*[xf-wb;yf;zf-radf];c2=q*[c2m(1,:)+wb;c2m(2,:);c2m(3,:)+radf];
              segments.c1=[c1(1,:)+x ; c1(2,:)+y ; c1(3,:)+z];
              segments.c2=[c2(1,:)+x ; c2(2,:)+y ; c2(3,:)+z];
            
              %Segment 1 (fork)
              p1s=[wb;0;radf];
            
              p1e=[-sin(lamda)*CD+wb;0;cos(lamda)*CD+radf];
            
              s1=q*[p1s p1e];
              segments.s1=[s1(1,:)+x ; s1(2,:)+y; s1(3,:)+z];
              %Segment 2 (frame)
            
              p2s=[0;0;0+radr];
            
              p2e=[-sin(lamda)*3/4*CD+wb;0;cos(lamda)*3/4*CD+radr];
            
              s2=q*[p2s p2e];
              segments.s2=[s2(1,:)+x ; s2(2,:)+y ; s2(3,:)+z];
            
              %Segment 3 (saddle frame)
            
              p3s=[-sin(lamda)*CD/4+wb/3;0;cos(lamda)*CD/4+radr];
            
              p3e=p3s;
              p3e(3)=p3s(3)+0.3;
            
              s3=q*[p3s p3e];
              segments.s3=[s3(1,:)+x ; s3(2,:)+y ; s3(3,:)+z];
            
              %Segment 4 (handlebar)
            
              p4s=[p1e(1)-0.1;p1e(2)-0.3;p1e(3)];
            
              p4l=[p1e(1);p1e(2)-0.3;p1e(3)];
            
              p4r=[p1e(1); p1e(2)+0.3 ;p1e(3)];
            
              p4e=[p1e(1)-0.1; p1e(2)+0.3 ;p1e(3)];
            
              s4=[p4s p4l p4r p4e];
              s4(3,:)=s4(3,:)-CD*cos(lamda)-radr;
              s4(2,:)=s4(2,:);
              s4(1,:)=s4(1,:)+CD*sin(lamda)-wb;
              s4=q2*s4;
              s4(3,:)=s4(3,:)+CD*cos(lamda)+radr;
              s4(2,:)=s4(2,:);
              s4(1,:)=s4(1,:)-CD*sin(lamda)+wb;
              s4=q*s4;
              segments.s4=[s4(1,:)+x ; s4(2,:)+y ; s4(3,:)+z];
            
              %Segment 5 (saddle)
            
              p5s=[p3e(1)-0.03;p3e(2)-0.07;p3e(3)];
            
              p5m=[p3e(1)+0.15; p3e(2); p3e(3)];
            
              p5e=[p3e(1)-0.03; p3e(2)+0.07; p3e(3)];
            
              s5=q*[p5s p5m p5e];
              segments.s5=[s5(1,:)+x ; s5(2,:)+y ; s5(3,:)+z];
            
              %Segment 6 (force vector)
              p6s=p3e;
            
              p6m=p6s+[0;-1*nw-0.01;0];
              p6l=p6m+[0.1*nw;0.08*nw;0];
              p6r=p6m+[-0.1*nw;0.08*nw;0];
            
              s6=q*[p6s p6m p6l p6r p6m];
              segments.s6=[s6(1,:)+x ; s6(2,:)+y ; s6(3,:)+z];
            
              %Segment 7 ( wheel rods)
               o7r=zeros(3,6);
               p7r=q*[o7r(1,:) ; o7r(2,:)   ;    o7r(3,:)+radr];
               segments.s7r=[p7r(1,:)+x ; p7r(2,:)+y ;    p7r(3,:)+z   ];
            
               p7f=q*[o7r(1,:)+wb ; o7r(2,:)   ;    o7r(3,:)+radr];
               segments.s7f=[p7f(1,:)+x ; p7f(2,:)+y ;    p7f(3,:)+z   ];
        end
        
        function updateChild(app)
            ye=app.state;   
            counter=0;
            idx=0;
            while(1)
                idx=idx+1;
                if(app.step<app.duration)
                    app.step=app.step+1;
                    if(app.step<floor(app.duration/2))
                       we=(app.magnitude*2/(app.duration*app.dt))*(app.dt*app.step);
                    else
                       we=-(app.magnitude*2/(app.duration*app.dt))*(app.dt*app.step)+2*app.magnitude;
                    end
                elseif(app.noise_flag==true)
                    we=abs(app.magnitude)*randn(1);
                else
                    we=0;
                    app.step=15000;
                end


                app.w=we;                
                [app.t,ye]=rk4step(app.t,ye,app.dt,app.v,app.Gp,app.K,we);               
                counter=counter+1;
                app.state=ye;
                app.phi=ye(3);app.delta=ye(4);app.psi=ye(5);app.xp=ye(8);app.yp=ye(9);app.Trider=ye(6);
                if (app.bike_plot_flag && app.subplot_flag)
                    frame_skip=8;
                else
                    frame_skip=4;
                end
                if (counter>frame_skip)
                    counter=0;
                    if(app.bike_plot_flag==true)
                        segments=updateSegments(app,app.t,we/abs(app.magnitude));
                        updateBike(app,segments)
                    end                                            
                    if ( app.subplot_flag==true)
                        updateSignals(app)
                    end
                end
                if(app.freeze==1)
                    break;
                end
                drawnow limitrate
            end
        end
        
        
        
        function updateBike(app,segments)
              set(app.p(1),'XData',segments.s1(1,:),'YData',segments.s1(2,:),'ZData',segments.s1(3,:))
              set(app.p(2),'XData',segments.c1(1,:),'YData',segments.c1(2,:),'ZData',segments.c1(3,:))
              set(app.p(3),'XData',segments.c2(1,:),'YData',segments.c2(2,:),'ZData',segments.c2(3,:))
              set(app.p(4),'XData',segments.s2(1,:),'YData',segments.s2(2,:),'ZData',segments.s2(3,:))
              set(app.p(5),'XData',segments.s3(1,:),'YData',segments.s3(2,:),'ZData',segments.s3(3,:))
              set(app.p(6),'XData',segments.s4(1,:),'YData',segments.s4(2,:),'ZData',segments.s4(3,:))
              for jj=10:10:60
                set(app.p(jj/10+8),'XData',[segments.s7r(1,jj/10) segments.c1(1,jj)],'YData',[segments.s7r(2,jj/10) segments.c1(2,jj)],'ZData',[segments.s7r(3,jj/10) segments.c1(3,jj)])
                set(app.rods(jj/10),'XData',[segments.s7f(1,jj/10) segments.c2(1,jj)],'YData',[segments.s7f(2,jj/10) segments.c2(2,jj)],'ZData',[segments.s7f(3,jj/10) segments.c2(3,jj)])
              end
              %addpoints(app.path_line,app.xp,app.yp,0)
              set(app.p(7),'XData',segments.s5(1,:),'YData',segments.s5(2,:),'ZData',segments.s5(3,:))
              set(app.p(8),'XData',segments.s6(1,:),'YData',segments.s6(2,:),'ZData',segments.s6(3,:))
              ylim(app.UIAxes,[app.yp-1.1 app.yp+1.1])
              xlim(app.UIAxes,[app.xp-1 app.xp+2.])
              if (app.camera=='1')
                view(app.UIAxes,app.azimuth,app.elevation)
              elseif(app.camera=='2')
                 view(app.UIAxes,-90,72)
              elseif (app.camera=='3')
                 view(app.UIAxes,-90,0)
  
              end
        end
        
        function  updateSignals(app)
            addpoints(app.roll_line,app.t,app.phi*180/pi);
            %hold(app.roll,'on')
            addpoints(app.steer_line,app.t,app.delta*180/pi)
            %hold(app.steer,'on')
            addpoints(app.torque_line,app.t,app.Trider)
            %hold(app.torque,'on')
        end
        
        function mod = getPlantModel(app)
            steerbyWireParam
            Drearwheel=app.r*2;
            Dfrontwheel=app.r*2;
            wheelbase=app.wheelbase;
            lambda=app.trail_angle;
            trail= (Dfrontwheel/2*sin(lambda)-0.044)/cos(lambda);
            JBmck %get matrices M0 C1 K0 K1 that fully desicribe the bicycle's equations of motion
            a = -fliplr(eye(2)) + eye(2);
            M0 = a .* M0;
            C1 = C1 .* a;
            K0 = K0 .* a;
            K2 = K2 .* a;
            Hfw = [0.84; 0.014408]; % dfx/dTq
            
            
            O = zeros(2);
            I = eye(2); % Some easy notations
            
            % State Space description of uncontrolled bicycle
            A = [-M0 \ C1 * app.v, -M0 \ (K0 + K2 * app.v^2); I, O];
            B = [M0 \ [I, Hfw]; zeros(2, 3)];
            
            E=[0 trail*cos(app.trail_angle)/app.wheelbase 0 app.v*cos(app.trail_angle)/app.wheelbase];
            NA=[A;E];
            NA=[NA zeros(5,1)];
            NB=[ B ;zeros(1,size(B,2))];
            NC=eye(5) ;
            ND=zeros(5,3);
            
            % Combine the extended whipple model with heading into a sys
            % object
            bike = ss(NA, NB, NC, ND);
            bike.u(1)=cellstr('Tlean'); %rider lean torque
            bike.u(2)=cellstr('Tdelta'); 
            bike.u(3)=cellstr('w'); %lateral force
            
            %Neuromuscular Dynamics

            Gnm.A=[0 1; -app.omegac^2 -2*sqrt(1/2)*app.omegac];
            Gnm.B=[0;app.omegac^2];
            Gnm.C=[1 0];
            Gnm.D=0;
            Gnm=ss(Gnm.A,Gnm.B,Gnm.C,Gnm.D);
            Gnm.u='a'; %neural input
            Gnm.y='Tdelta';%rider steer torque
            

            
            bike.y='y';
            input={'Tlean';'a';'w'};
            mod=connect(bike,Gnm,input,'y');

        end
        
        function rm = rotmat(app,uv,angle)
            s=[0 -uv(3) uv(2);uv(3) 0 -uv(1);-uv(2) uv(1) 0];
            rm=cos(angle)*eye(3)+(1-cos(angle))*(uv)*uv'+sin(angle)*s;
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function constructor(app)
             %set initial values to member variables
             app.theta=linspace(0,2*pi,60);%rad
             app.v=9.3/3.6;
             app.omegac= 2 * pi * 2.17;
             app.trail_angle=deg2rad(90-73);
             app.duration=20;
             app.magnitude=350;
             app.step=app.duration;
             app.wheelbase=1.03;
             app.r=0.6858/2;
             app.dt=0.01;
             app.t=0;
             app.subplot_flag=false;
             app.bike_plot_flag=true;
             app.azimuth=60;
             app.elevation=30;
             %create subplots
             app.roll.Visible='off';
             app.steer.Visible='off';
             app.torque.Visible='off';
             app.roll_line=animatedline(app.roll,'Color','b','MaximumNumPoints',100);
             app.steer_line=animatedline(app.steer,'Color','b','MaximumNumPoints',100);
             app.torque_line=animatedline(app.torque,'Color','b','MaximumNumPoints',100);
             app.roll.cla;
             app.steer.cla;
             app.torque.cla;
             %set default values to buttons
             app.DurationmsEditField.Value = 200;
             app.SignalsSwitch.Value='Off';
             app.ControllerSwitch.Value = 'On';
             app.WheelbasecmEditField.Value = 103;
             app.AzimuthSlider.Value = 60;
             app.ElevationSlider.Value = 30;
             app.MagnitudeNEditField.Value = 350;
             app.SteerAxisTiltdegSlider.Value = 17;
             app.WheelRadiuscmEditField.Value = 34.29;
             app.IntegrationTimeStepSpinner.Value = 0.01;
             % create proper plot labels and grids
             app.steer.XGrid="on";
             app.steer.YGrid="on";
             app.torque.XGrid="on";
             app.torque.YGrid="on";
             app.roll.XGrid="on";
             app.roll.YGrid="on";
             app.steer.YLabel.String="\delta (deg)";
             app.torque.YLabel.String="T_{\delta} (Nm)";   
             app.roll.YLabel.String="\phi (deg)"; 
             % initialize the model
             Y=load('gains_6param_steer.mat');
             app.K=Y.Gains(1,:);
             app.UITable.Data=app.K;
             app.Gp=getPlantModel(app);
             app.xp=0;app.yp=0;app.psi=0;app.phi=0;app.delta=0;
             app.state=zeros(9,1);
             segments=updateSegments(app,0,0);
             plotBike(app,segments);
             %enter update loop function
             updateChild(app);
        end

        % Button pushed function: PauseButton
        function PauseButtonPushed(app, event)
        if(app.freeze==0)
             app.PauseButton.Text = 'Resume';
             app.freeze=1;            
        else
            app.PauseButton.Text = 'Pause';
            app.freeze=0;
            updateChild(app)           
        end
           
        end

        % Button pushed function: ImpulseButton
        function ImpulseButtonPushed(app, event)
                app.step=0;
        end

        % Value changed function: CameraViewDropDown
        function CameraViewDropDownValueChanged(app, event)
            value = app.CameraViewDropDown.Value;
            app.camera=value;
            if(value=='1')
                app.AzimuthSlider.Visible="on";
                app.ElevationSlider.Visible="on";
                app.AzimuthSliderLabel.Text="Azimuth";
                app.ElevationSliderLabel.Text="Elevation";

            else
                app.AzimuthSlider.Visible="off";
                app.ElevationSlider.Visible="off";
                app.AzimuthSliderLabel.Text="";
                app.ElevationSliderLabel.Text="";

            end
        end

        % Value changed function: SignalsSwitch
        function SignalsSwitchValueChanged(app, event)
            value = app.SignalsSwitch.Value;
            if (value=="On")
                app.subplot_flag=true;
                app.roll.Visible='on';
                app.steer.Visible='on';
                app.torque.Visible='on';
                app.roll_line=animatedline(app.roll,'Color','b','MaximumNumPoints',100);
                app.steer_line=animatedline(app.steer,'Color','b','MaximumNumPoints',100);
                app.torque_line=animatedline(app.torque,'Color','b','MaximumNumPoints',100);
            else
                app.subplot_flag=false;
                app.roll.cla;
                app.steer.cla;
                app.torque.cla;

                app.roll.Visible='off';
                app.steer.Visible='off';
                app.torque.Visible='off';

            end
        end

        % Value changed function: MagnitudeNEditField
        function MagnitudeNEditFieldValueChanged(app, event)
            value = app.MagnitudeNEditField.Value;
            app.magnitude=value;
        end

        % Value changed function: DurationmsEditField
        function DurationmsEditFieldValueChanged(app, event)
            value = app.DurationmsEditField.Value;
            app.duration=value/1000/app.dt;
        end

        % Value changed function: ForwardSpeedkmhDropDown
        function ForwardSpeedkmhDropDownValueChanged(app, event)
            value = app.ForwardSpeedkmhDropDown.Value;
            app.v=str2double(value)/3.6;
            if(app.v<3)
                n=1;
            elseif(app.v<4)
                n=2;
            elseif(app.v<5)
                n=3;
            else
                n=4;
            end
            Y=load('gains_6param_steer.mat');
            app.K=Y.Gains(n,:);
            app.UITable.Data=app.K;
            app.Gp=getPlantModel(app);
            app.ControllerSwitch.Value = 'On';
        end

        % Value changing function: IntegrationTimeStepSpinner
        function IntegrationTimeStepSpinnerValueChanging(app, event)
            changingValue = event.Value;
            x=app.duration*app.dt;
            app.dt=changingValue;
            app.duration=x/app.dt;
        end

        % Value changed function: Switch
        function SwitchValueChanged(app, event)
            value = app.Switch.Value;
            if (value=="On")
                app.bike_plot_flag=true;
                segments=updateSegments(app,app.t,app.w/abs(app.magnitude));
                plotBike(app,segments)
                app.UIAxes.Visible="on";
            else
                app.bike_plot_flag=false;
                app.UIAxes.Visible="off";
                app.UIAxes.cla;
            end
        end

        % Value changing function: SteerAxisTiltdegSlider
        function SteerAxisTiltdegSliderValueChanging(app, event)
            changingValue = event.Value;
            app.trail_angle = changingValue*pi/180;
            app.Gp=getPlantModel(app);
        end

        % Value changed function: WhiteNoiseSwitch
        function WhiteNoiseSwitchValueChanged(app, event)
            value = app.WhiteNoiseSwitch.Value;
            if (value=="On")
                app.noise_flag=true;
            else
                app.noise_flag=false;
            end
        end

        % Value changing function: AzimuthSlider
        function AzimuthSliderValueChanging(app, event)
            changingValue = event.Value;
            app.azimuth=changingValue;
        end

        % Value changing function: ElevationSlider
        function ElevationSliderValueChanging(app, event)
            changingValue = event.Value;
            app.elevation=changingValue;
        end

        % Value changed function: WheelbasecmEditField
        function WheelbasecmEditFieldValueChanged(app, event)
            value = app.WheelbasecmEditField.Value;
            app.wheelbase=value/100;
            app.Gp=getPlantModel(app);
        end

        % Value changed function: WheelRadiuscmEditField
        function WheelRadiuscmEditFieldValueChanged(app, event)
            value = app.WheelRadiuscmEditField.Value;
            app.r=value/100;
            app.Gp=getPlantModel(app);
        end

        % Value changed function: ControllerSwitch
        function ControllerSwitchValueChanged(app, event)
            value = app.ControllerSwitch.Value;
            if (value=="Off")
                app.K=zeros(1,6);
                app.UITable.Data=app.K;
            else
                if(app.v<3)
                    n=1;
                elseif(app.v<4)
                    n=2;
                elseif(app.v<5)
                    n=3;
                else
                    n=4;
                end
    Y=load('gains_6param_steer.mat');
                app.K=Y.Gains(n).K;
                app.UITable.Data=app.K;
            end
        end

        % Button pushed function: ResetButton
        function ResetButtonPushed(app, event)
            constructor(app)
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create BicycleRiderSimulationUIFigure and hide until all components are created
            app.BicycleRiderSimulationUIFigure = uifigure('Visible', 'off');
            app.BicycleRiderSimulationUIFigure.Color = [0.9412 0.9412 0.9412];
            app.BicycleRiderSimulationUIFigure.Colormap = [0.2431 0.149 0.6588;0.251 0.1647 0.7059;0.2588 0.1804 0.7529;0.2627 0.1961 0.7961;0.2706 0.2157 0.8353;0.2745 0.2353 0.8706;0.2784 0.2549 0.898;0.2784 0.2784 0.9216;0.2824 0.302 0.9412;0.2824 0.3216 0.9569;0.2784 0.3451 0.9725;0.2745 0.3686 0.9843;0.2706 0.3882 0.9922;0.2588 0.4118 0.9961;0.2431 0.4353 1;0.2196 0.4588 0.9961;0.1961 0.4863 0.9882;0.1843 0.5059 0.9804;0.1804 0.5294 0.9686;0.1765 0.549 0.9529;0.1686 0.5686 0.9373;0.1529 0.5922 0.9216;0.1451 0.6078 0.9098;0.1373 0.6275 0.898;0.1255 0.6471 0.8902;0.1098 0.6627 0.8745;0.0941 0.6784 0.8588;0.0706 0.6941 0.8392;0.0314 0.7098 0.8157;0.0039 0.7216 0.7922;0.0078 0.7294 0.7647;0.0431 0.7412 0.7412;0.098 0.749 0.7137;0.1412 0.7569 0.6824;0.1725 0.7686 0.6549;0.1922 0.7765 0.6235;0.2157 0.7843 0.5922;0.2471 0.7922 0.5569;0.2902 0.7961 0.5176;0.3412 0.8 0.4784;0.3922 0.8039 0.4353;0.4471 0.8039 0.3922;0.5059 0.8 0.349;0.5608 0.7961 0.3059;0.6157 0.7882 0.2627;0.6706 0.7804 0.2235;0.7255 0.7686 0.1922;0.7725 0.7608 0.1647;0.8196 0.749 0.1529;0.8627 0.7412 0.1608;0.902 0.7333 0.1765;0.9412 0.7294 0.2118;0.9725 0.7294 0.2392;0.9961 0.7451 0.2353;0.9961 0.7647 0.2196;0.9961 0.7882 0.2039;0.9882 0.8118 0.1882;0.9804 0.8392 0.1765;0.9686 0.8627 0.1647;0.9608 0.8902 0.1529;0.9608 0.9137 0.1412;0.9647 0.9373 0.1255;0.9686 0.9608 0.1059;0.9765 0.9843 0.0824];
            app.BicycleRiderSimulationUIFigure.Position = [100 100 993 661];
            app.BicycleRiderSimulationUIFigure.Name = 'Bicycle Rider Simulation';

            % Create UIAxes
            app.UIAxes = uiaxes(app.BicycleRiderSimulationUIFigure);
            title(app.UIAxes, '')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.Position = [47 266 421 353];

            % Create PauseButton
            app.PauseButton = uibutton(app.BicycleRiderSimulationUIFigure, 'push');
            app.PauseButton.ButtonPushedFcn = createCallbackFcn(app, @PauseButtonPushed, true);
            app.PauseButton.Position = [817 56 100 22];
            app.PauseButton.Text = 'Pause';

            % Create ImpulseButton
            app.ImpulseButton = uibutton(app.BicycleRiderSimulationUIFigure, 'push');
            app.ImpulseButton.ButtonPushedFcn = createCallbackFcn(app, @ImpulseButtonPushed, true);
            app.ImpulseButton.Position = [47 138 100 22];
            app.ImpulseButton.Text = 'Impulse';

            % Create CameraViewDropDownLabel
            app.CameraViewDropDownLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.CameraViewDropDownLabel.HorizontalAlignment = 'right';
            app.CameraViewDropDownLabel.Position = [803 256 77 22];
            app.CameraViewDropDownLabel.Text = 'Camera View';

            % Create CameraViewDropDown
            app.CameraViewDropDown = uidropdown(app.BicycleRiderSimulationUIFigure);
            app.CameraViewDropDown.Items = {'3D', 'Top Down', 'Back'};
            app.CameraViewDropDown.ItemsData = {'1', '2', '3'};
            app.CameraViewDropDown.ValueChangedFcn = createCallbackFcn(app, @CameraViewDropDownValueChanged, true);
            app.CameraViewDropDown.Position = [792 225 100 22];
            app.CameraViewDropDown.Value = '1';

            % Create roll
            app.roll = uiaxes(app.BicycleRiderSimulationUIFigure);
            title(app.roll, '')
            xlabel(app.roll, '')
            ylabel(app.roll, '')
            app.roll.Position = [543 400 376 97];

            % Create torque
            app.torque = uiaxes(app.BicycleRiderSimulationUIFigure);
            title(app.torque, '')
            xlabel(app.torque, '')
            ylabel(app.torque, '')
            app.torque.Position = [543 496 376 103];

            % Create steer
            app.steer = uiaxes(app.BicycleRiderSimulationUIFigure);
            title(app.steer, '')
            xlabel(app.steer, 'Time (s)')
            ylabel(app.steer, '')
            app.steer.Position = [543 304 376 97];

            % Create SignalsSwitchLabel
            app.SignalsSwitchLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.SignalsSwitchLabel.HorizontalAlignment = 'center';
            app.SignalsSwitchLabel.FontSize = 14;
            app.SignalsSwitchLabel.FontWeight = 'bold';
            app.SignalsSwitchLabel.Position = [664 618 55 22];
            app.SignalsSwitchLabel.Text = 'Signals';

            % Create SignalsSwitch
            app.SignalsSwitch = uiswitch(app.BicycleRiderSimulationUIFigure, 'slider');
            app.SignalsSwitch.ValueChangedFcn = createCallbackFcn(app, @SignalsSwitchValueChanged, true);
            app.SignalsSwitch.Position = [753 624 26 11];

            % Create MagnitudeNEditFieldLabel
            app.MagnitudeNEditFieldLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.MagnitudeNEditFieldLabel.HorizontalAlignment = 'right';
            app.MagnitudeNEditFieldLabel.Position = [27 204 82 22];
            app.MagnitudeNEditFieldLabel.Text = 'Magnitude (N)';

            % Create MagnitudeNEditField
            app.MagnitudeNEditField = uieditfield(app.BicycleRiderSimulationUIFigure, 'numeric');
            app.MagnitudeNEditField.ValueChangedFcn = createCallbackFcn(app, @MagnitudeNEditFieldValueChanged, true);
            app.MagnitudeNEditField.Tooltip = {'Changes the peak of the impulse. In the case of white noise perturbation it determines the variance of the random signal.'};
            app.MagnitudeNEditField.Position = [116 204 41 22];
            app.MagnitudeNEditField.Value = 350;

            % Create DisturbanceLabel
            app.DisturbanceLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.DisturbanceLabel.FontSize = 14;
            app.DisturbanceLabel.FontWeight = 'bold';
            app.DisturbanceLabel.Position = [61 235 87 22];
            app.DisturbanceLabel.Text = 'Disturbance';

            % Create DurationmsEditFieldLabel
            app.DurationmsEditFieldLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.DurationmsEditFieldLabel.HorizontalAlignment = 'right';
            app.DurationmsEditFieldLabel.Position = [31 169 78 22];
            app.DurationmsEditFieldLabel.Text = 'Duration (ms)';

            % Create DurationmsEditField
            app.DurationmsEditField = uieditfield(app.BicycleRiderSimulationUIFigure, 'numeric');
            app.DurationmsEditField.ValueChangedFcn = createCallbackFcn(app, @DurationmsEditFieldValueChanged, true);
            app.DurationmsEditField.Tooltip = {'Duration of the impulse perturbation'};
            app.DurationmsEditField.Position = [116 169 41 22];
            app.DurationmsEditField.Value = 200;

            % Create ForwardSpeedkmhDropDownLabel
            app.ForwardSpeedkmhDropDownLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.ForwardSpeedkmhDropDownLabel.HorizontalAlignment = 'right';
            app.ForwardSpeedkmhDropDownLabel.Position = [327 204 125 22];
            app.ForwardSpeedkmhDropDownLabel.Text = 'Forward Speed (km/h)';

            % Create ForwardSpeedkmhDropDown
            app.ForwardSpeedkmhDropDown = uidropdown(app.BicycleRiderSimulationUIFigure);
            app.ForwardSpeedkmhDropDown.Items = {'9', '13', '16', '20'};
            app.ForwardSpeedkmhDropDown.ValueChangedFcn = createCallbackFcn(app, @ForwardSpeedkmhDropDownValueChanged, true);
            app.ForwardSpeedkmhDropDown.Position = [467 204 100 22];
            app.ForwardSpeedkmhDropDown.Value = '9';

            % Create UITable
            app.UITable = uitable(app.BicycleRiderSimulationUIFigure);
            app.UITable.ColumnName = {'K1'; 'K2'; 'K3'; 'K4'; 'K5'; 'K6'};
            app.UITable.ColumnWidth = {'auto'};
            app.UITable.RowName = {};
            app.UITable.ColumnEditable = false;
            app.UITable.Tooltip = {'The six gains each correcting the error of  the states. '; ' K1'; ' K2'; ' K3'; ' K4'; ' K5'; ' K6 '; 'close the error in '; 'roll rate'; 'steer rate'; 'roll angle'; 'steer angle'; 'yaw angle and applied rider torque respectively.'};
            app.UITable.FontWeight = 'bold';
            app.UITable.FontSize = 10;
            app.UITable.Position = [217 138 452 58];

            % Create ModelParametersLabel
            app.ModelParametersLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.ModelParametersLabel.FontSize = 14;
            app.ModelParametersLabel.FontWeight = 'bold';
            app.ModelParametersLabel.Position = [403 235 126 22];
            app.ModelParametersLabel.Text = 'Model Parameters';

            % Create IntegrationTimeStepSpinnerLabel
            app.IntegrationTimeStepSpinnerLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.IntegrationTimeStepSpinnerLabel.HorizontalAlignment = 'right';
            app.IntegrationTimeStepSpinnerLabel.Position = [275 110 119 22];
            app.IntegrationTimeStepSpinnerLabel.Text = 'Integration Time Step';

            % Create IntegrationTimeStepSpinner
            app.IntegrationTimeStepSpinner = uispinner(app.BicycleRiderSimulationUIFigure);
            app.IntegrationTimeStepSpinner.Step = 0.001;
            app.IntegrationTimeStepSpinner.ValueChangingFcn = createCallbackFcn(app, @IntegrationTimeStepSpinnerValueChanging, true);
            app.IntegrationTimeStepSpinner.Tooltip = {'Time step of integration ( Runke-Kuta 4)'};
            app.IntegrationTimeStepSpinner.Position = [285 89 100 22];
            app.IntegrationTimeStepSpinner.Value = 0.01;

            % Create Switch
            app.Switch = uiswitch(app.BicycleRiderSimulationUIFigure, 'slider');
            app.Switch.ValueChangedFcn = createCallbackFcn(app, @SwitchValueChanged, true);
            app.Switch.Position = [321 623 28 12];
            app.Switch.Value = 'On';

            % Create BicycleAnimationLabel
            app.BicycleAnimationLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.BicycleAnimationLabel.FontSize = 14;
            app.BicycleAnimationLabel.FontWeight = 'bold';
            app.BicycleAnimationLabel.Position = [165 618 127 22];
            app.BicycleAnimationLabel.Text = 'Bicycle Animation';

            % Create SteerAxisTiltdegSliderLabel
            app.SteerAxisTiltdegSliderLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.SteerAxisTiltdegSliderLabel.HorizontalAlignment = 'right';
            app.SteerAxisTiltdegSliderLabel.Position = [276 56 111 22];
            app.SteerAxisTiltdegSliderLabel.Text = 'Steer Axis Tilt (deg)';

            % Create SteerAxisTiltdegSlider
            app.SteerAxisTiltdegSlider = uislider(app.BicycleRiderSimulationUIFigure);
            app.SteerAxisTiltdegSlider.Limits = [-30 30];
            app.SteerAxisTiltdegSlider.ValueChangingFcn = createCallbackFcn(app, @SteerAxisTiltdegSliderValueChanging, true);
            app.SteerAxisTiltdegSlider.Tooltip = {'Angle measured from the vertical z axis to the axis defined by the steering assembly.'};
            app.SteerAxisTiltdegSlider.Position = [276 48 119 3];
            app.SteerAxisTiltdegSlider.Value = 17;

            % Create WhiteNoiseSwitchLabel
            app.WhiteNoiseSwitchLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.WhiteNoiseSwitchLabel.HorizontalAlignment = 'center';
            app.WhiteNoiseSwitchLabel.Position = [27 107 70 22];
            app.WhiteNoiseSwitchLabel.Text = 'White Noise';

            % Create WhiteNoiseSwitch
            app.WhiteNoiseSwitch = uiswitch(app.BicycleRiderSimulationUIFigure, 'slider');
            app.WhiteNoiseSwitch.ValueChangedFcn = createCallbackFcn(app, @WhiteNoiseSwitchValueChanged, true);
            app.WhiteNoiseSwitch.Position = [134 109 41 18];

            % Create AzimuthSliderLabel
            app.AzimuthSliderLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.AzimuthSliderLabel.HorizontalAlignment = 'right';
            app.AzimuthSliderLabel.Position = [747 194 49 22];
            app.AzimuthSliderLabel.Text = 'Azimuth';

            % Create AzimuthSlider
            app.AzimuthSlider = uislider(app.BicycleRiderSimulationUIFigure);
            app.AzimuthSlider.Limits = [-90 90];
            app.AzimuthSlider.ValueChangingFcn = createCallbackFcn(app, @AzimuthSliderValueChanging, true);
            app.AzimuthSlider.Position = [817 203 111 3];
            app.AzimuthSlider.Value = 60;

            % Create ElevationSliderLabel
            app.ElevationSliderLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.ElevationSliderLabel.HorizontalAlignment = 'right';
            app.ElevationSliderLabel.Position = [747 137 55 22];
            app.ElevationSliderLabel.Text = 'Elevation';

            % Create ElevationSlider
            app.ElevationSlider = uislider(app.BicycleRiderSimulationUIFigure);
            app.ElevationSlider.Limits = [-90 90];
            app.ElevationSlider.ValueChangingFcn = createCallbackFcn(app, @ElevationSliderValueChanging, true);
            app.ElevationSlider.Position = [817 146 111 3];
            app.ElevationSlider.Value = 30;

            % Create WheelRadiuscmEditFieldLabel
            app.WheelRadiuscmEditFieldLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.WheelRadiuscmEditFieldLabel.HorizontalAlignment = 'right';
            app.WheelRadiuscmEditFieldLabel.Position = [414 110 107 22];
            app.WheelRadiuscmEditFieldLabel.Text = 'Wheel Radius (cm)';

            % Create WheelRadiuscmEditField
            app.WheelRadiuscmEditField = uieditfield(app.BicycleRiderSimulationUIFigure, 'numeric');
            app.WheelRadiuscmEditField.ValueChangedFcn = createCallbackFcn(app, @WheelRadiuscmEditFieldValueChanged, true);
            app.WheelRadiuscmEditField.Position = [536 110 100 22];
            app.WheelRadiuscmEditField.Value = 34.29;

            % Create WheelbasecmEditFieldLabel
            app.WheelbasecmEditFieldLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.WheelbasecmEditFieldLabel.HorizontalAlignment = 'right';
            app.WheelbasecmEditFieldLabel.Position = [428 77 93 22];
            app.WheelbasecmEditFieldLabel.Text = 'Wheelbase (cm)';

            % Create WheelbasecmEditField
            app.WheelbasecmEditField = uieditfield(app.BicycleRiderSimulationUIFigure, 'numeric');
            app.WheelbasecmEditField.ValueChangedFcn = createCallbackFcn(app, @WheelbasecmEditFieldValueChanged, true);
            app.WheelbasecmEditField.Tooltip = {'Distance between the centers of the wheels.'};
            app.WheelbasecmEditField.Position = [536 77 100 22];
            app.WheelbasecmEditField.Value = 103;

            % Create ControllerSwitchLabel
            app.ControllerSwitchLabel = uilabel(app.BicycleRiderSimulationUIFigure);
            app.ControllerSwitchLabel.HorizontalAlignment = 'center';
            app.ControllerSwitchLabel.Position = [450 38 58 22];
            app.ControllerSwitchLabel.Text = 'Controller';

            % Create ControllerSwitch
            app.ControllerSwitch = uiswitch(app.BicycleRiderSimulationUIFigure, 'slider');
            app.ControllerSwitch.ValueChangedFcn = createCallbackFcn(app, @ControllerSwitchValueChanged, true);
            app.ControllerSwitch.Tooltip = {'Turn control off. Explore the self stability of the bicycle at the 4th speed level.'};
            app.ControllerSwitch.Position = [545 44 22 10];
            app.ControllerSwitch.Value = 'On';

            % Create ResetButton
            app.ResetButton = uibutton(app.BicycleRiderSimulationUIFigure, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.Position = [817 26 100 22];
            app.ResetButton.Text = 'Reset';

            % Show the figure after all components are created
            app.BicycleRiderSimulationUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = simulation_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.BicycleRiderSimulationUIFigure)

            % Execute the startup function
            runStartupFcn(app, @constructor)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.BicycleRiderSimulationUIFigure)
        end
    end
end