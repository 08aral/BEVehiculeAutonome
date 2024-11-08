clf;
clear all;

%% parametres
xr0=5;
yr0=5;
theta0 = asin( pi / 8);
L=0.2;
xr=xr0;
yr=yr0;
thetar=theta0;



%% Simulation

simu = sim('AutomatiqueCoursSimuL2.slx');

xrpoint = simu.xrpoint;
yrpoint = simu.yrpoint;
thetarpoint = simu.thetap;
pol = simu.pol
polderive = simu.polderive


% xrT = xr + xrpoint(1)
% yrT = yr + yrpoint(1)
% thetarT = thetar + thetarpoint(1) %mettre en timeseries pour tracer e nfct du temps sinon array sert a obtenir vecteur de valeurs ds le temps (ideal pour tracer x en fct de y)
% 
% 
% xr = xrT(-1)
% yr = yrT(-1)
% thetar = thetarT(-1)
%% Visualisation

% plot(simu.x);