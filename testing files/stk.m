close all;clear all;clc
% This software was written by Capt Barry Witt in Dec 2008. In general, it
% displays graphically the computed delta V's for an interceptor and a target
% using TLE's for the satellites and a user defined search window. The
% user defines one window for the interceptor (when the burn will take
% place) and one to many windows for the target (when the interception will
% take place). For each burn/intercept combination a delta V is calculated
% into an array, and the array is displayed as a surface plot. Further,
% multiple targets can be looked at in succession. Please note that for
% the new interceptor orbit, only the 6 COEs and epoch time are updated.
% The software can be divided up into 4 parts. The first part loads the
% TLEs into the SGP4 propagator. The second part creates the burn and
% intercept windows. The third part calculates and displays the delta Vs.
% The fourth part finds the intercept time and classical orbit elements of
% the new orbit and then updates the interceptor TLE with the new epoch
% time and COEs. It then loops it over again.
% Feature To-Do list
% 1. Add the option for repeated intercepts
% 2. Simplify initialization of SGP4
% Bug To-Do List
% 1.
disp(' ')
disp('Welcome to the Mission Planning Tool for Satellites')
disp('Please verify TLE data has been properly inserted into this m-file')
numtgts=input('How many targets will be looked at? ');
total_time=0; %this variable keep tracks of the total time since original
epoch of the interceptor.
total_burn=0; %this variable keep tracks of the total total delta V.
whichconst=84;typerun='m';typeinput='d';%These are variables for twoline2rv.m
Using '84 earth model, with a manual input'
DU2Km=6378.135;
TU2min=13.44686457;
VU2KmSec=7.90536828;
Re=1+200/6378; %this is the radius of the earth plus a safety factor of 200km.
38
%------------------------Place Interceptor TLE Below-----------------------
%using longstr1 = first line and longstr2 = second line
%XSS-11
longstr1='1 99991U 09011.00000000 -.00000650 00000-0 -36969-3 0 00000';
longstr2='2 99991 098.7964 000.1055 0010201 271.5869 088.4544 14.09867153000011';
%Sat 1 180 off phase
% longstr1='1 99992U 07182.50000000 -.00000812 00000-0 -34300-4 0
00008';
% longstr2='2 99992 045.0242 000.0567 0007672 267.2596 272.7984
15.22470883000011';
%Sat1 different inclinations
% longstr1='1 99994U 07182.50000000 -.00009755 00000-0 -21362-5 0
00008';
% longstr2='2 99994 023.0210 000.0271 0004507 262.3204 098.2633
16.29163400000014';
disp(' ')
disp('The propogator will now initialize.')
disp('Please select dates that will cover the entire time period under consideration.')
disp(' ')
disp('For the interceptor')
[satrec(1,1), startmfe, stopmfe, deltamin] = twoline2rv(whichconst, longstr1, longstr2, typerun,typeinput); %Interceptor at satrec(1,1)
%------------------------Place Target(s) TLE Data Below--------------------
% Use naming convetion for TLEs as longstrx and longstrx+1 where x is 2
% times the target number plus 1.
%Target 1 TLE Data
39
longstr3='1 28494U 04049C 09012.18614689 .00000013 00000-0 10668-4 0 6209';
longstr4='2 28494 98.2341 329.4839 0003441 28.7950 331.3451 14.70161446218169';
disp(' ')
disp('For Target 1')
[satrec(1,2), startmfe, stopmfe, deltamin] = twoline2rv(whichconst, longstr3, longstr4, typerun,typeinput); %Target 1 at satrec(1,2)
%Target 2 TLE Data
longstr5='1 26102U 00014A 09012.12933296 .00000597 00000-0 38532-4 0 2330';
longstr6='2 26102 97.2007 185.0417 0018749 295.1236 127.6792 15.11650443485822';
disp(' ')
disp('For Target 2')
[satrec(1,3), startmfe, stopmfe, deltamin] = twoline2rv(whichconst, longstr5, longstr6, typerun,typeinput); %Target 2 at satrec(1,3)
%Target 3 TLE Data
% longstr7='1 33442U 98067BL 09011.25286709 .00032848 00000-0 21202-3 0
846';
% longstr8='2 33442 51.6442 29.2806 0003042 156.4997 203.6516 15.75798010
8270';
%
%
% disp(' ')
% disp('For Target 3')
% [satrec(1,4), startmfe, stopmfe, deltamin] = twoline2rv(whichconst, longstr7, longstr8, typerun,typeinput); %Target 3 at satrec(1,4)
%---------------------------End of Target TLE Data------------------------
40
for tgtctr=1:numtgts %this is the begenning of the loop for each target
disp(' ') %this creates the burn window
disp('Please select dates and times that cooincide with the information entered above.')
disp(' ')
disp('For the interceptor, for the burn window.')
i_ep=input(['How long after day ',num2str(satrec(1,1).epochdays),' in year ',num2str(satrec(1,1).epochyr),' should the window begin? (in minutes) ']);
%FIX this to reflect the time of intercept from the previous burn
i_win=input('How long should the window be? (in minutes) ');
i_tstep=input('What time step should be used within the window? (in minutes) ');
i_blocks=floor(i_win/i_tstep); %this divides the window up into blocks, from
which to calculate the delta Vs.
for ctr=1:i_blocks+1
tsince(ctr)=i_ep+(ctr-1)*i_tstep;
[satrec(1,1), r_xss(:,ctr), v_xss(:,ctr)] = sgp4(satrec(1,1),tsince(ctr)); %generate r and v for the interceptor
end
disp(' ')
disp(['For target ',num2str(tgtctr),', for the intercept window.'])
nwin=input('How many intercept windows? '); %this allows multuple intercept
windows
for nctr=1:nwin %this is the begenning of the creation loop for intercept
windows
disp(' ')
disp(['For intercept window ',num2str(nctr),'.'])
t_ep(nctr)=input(['How long after day ',num2str(satrec(1,tgtctr+1).epochdays),' in year ',num2str(satrec(1,tgtctr+1).epochyr),' should the window begin? (in minutes) ']);
t_win(nctr)=input('How long should the window be? (in minutes) ');
41
t_tstep(nctr)=input('What time step should be used within the window? (in minutes) '); %this is similar to creating the burn window above.
t_blocks(nctr)=floor(t_win(nctr)/t_tstep(nctr));
for ctr2= 1:t_blocks(nctr)+1
tsince2(ctr2,nctr)=t_ep(nctr)+(ctr2-1)*t_tstep(nctr);
[satrec(1,tgtctr+1), r_hst(:,ctr2,nctr),v_hst(:,ctr2,nctr)] = sgp4(satrec(1,tgtctr+1),tsince2(ctr2,nctr)); %generate r and v for the
target
end
deltaep=365-satrec(1,1).epochdays+((satrec(1,tgtctr+1).epochyrsatrec( 1,1).epochyr)-1)*365+satrec(1,tgtctr+1).epochdays; %this is the number of days between the TLE epochs of the interceptor and the target
for ctr3=1:i_blocks+1
for ctr4=1:t_blocks(nctr)+1
tdays(ctr3,ctr4,nctr)=deltaep+tsince2(ctr4,nctr)/(60*24)- tsince(ctr3)/(60*24); %Time of flight in days between each time step in the burn and intercept windows.
end
end
end
r_xss=r_xss/DU2Km; %Piter works only with Canonical Units
v_xss=v_xss/VU2KmSec;
r_hst=r_hst/DU2Km;
v_hst=v_hst/VU2KmSec;
i_energy=0.5*(norm(v_xss(:,1)))^2-1/norm(r_xss(:,1));
i_a=-1/(2*i_energy);
period=(2*pi*sqrt(i_a^3))*TU2min; % this is the period of the interceptor in minutes/orbit of the pre-burn orbit.
nrev=round(tdays/(period/(24*60))); %nrev is the number of revolutions of the proposed interception orbit for piter. This is the number of revolutions the interceptor would take if it's orbital period did not change.
% the code below calculates the delta Vs for all of the burn and intercept
% combinations.
42
for nctr=1:nwin %this is the begenning of the loop for each intercept window
for ctr5=1:i_blocks+1 %this counter advances the interceptor
for ctr6=1:t_blocks(nctr)+1 %this advances the target
if tdays(ctr5,ctr6,nctr) < 0
mdelv(ctr5,ctr6,nctr)=NaN; %if the windows overlap, then some of
the combinations will have a negative tof
else
for ctrc=1:5 %this is the nrev counter
if nrev(ctr5,ctr6,nctr)-3+ctrc < 0 %if the nrev that is
proposed is less then zero, it needs to be skipped
mdelv3(ctrc)=NaN;
mdelv5(ctrc)=NaN;
continue
end
[nsoln,vr_xss,v21]=piter(r_xss(:,ctr5),r_hst(:,ctr6,nctr),tdays(ctr5,ctr6,nctr),0,nrev(ctr5,ctr6,nctr)-3+ctrc);
[nsoln2,vr_xss2,v22]=piter(r_xss(:,ctr5),r_hst(:,ctr6,nctr),tdays(ctr5,ctr6,nctr),1,nrev(ctr5,ctr6,nctr)-3+ctrc); %piter needs to be run in short way and
long way modes
if nsoln < eps %this code checks to see if piter returned
an actual vector or not
mdelv3(ctrc)=NaN;
end
if nsoln2 < eps
mdelv5(ctrc)=NaN;
end
if nsoln < eps && nsoln2 < eps %if both piter runs contain no solutions, continue to the next nrev
continue
end
for stu=1:nsoln
energy=0.5*(norm(vr_xss(:,stu)))^2-1/norm((r_xss(:,ctr5)));
a=-1/(2*energy);
H=cross(r_xss(:,ctr5),vr_xss(:,stu));
e=cross(vr_xss(:,stu),H)-r_xss(:,ctr5)/norm(r_xss(:,ctr5));
rp=a*(1-e);
if rp-Re < 0 %this codes checks to see if the perigee
height is above earth or not
vr_xss(:,stu)=[NaN;NaN;NaN];
end
delv2(:,stu)=vr_xss(:,stu)-v_xss(:,ctr5);
mdelv2(stu)=norm(delv2(:,stu));
end
43
if nsoln > eps %this prevents mdelv3 from being re-defined
if nsoln was 0.
mdelv3(ctrc)=min(mdelv2);
end
for stu=1:nsoln2
energy=0.5*(norm(vr_xss2(:,stu)))^2-1/norm((r_xss(:,ctr5)));
a=-1/(2*energy);
H=cross(r_xss(:,ctr5),vr_xss2(:,stu));
e=cross(vr_xss2(:,stu),H)-r_xss(:,ctr5)/norm(r_xss(:,ctr5));
rp=a*(1-e);
if rp-Re < 0 %this codes checks to see if the perigee
height is above earth or not
vr_xss2(:,stu)=[NaN;NaN;NaN];
end
delv4(:,stu)=vr_xss2(:,stu)-v_xss(:,ctr5);
mdelv4(stu)=norm(delv4(:,stu));
end
if nsoln2 > eps %this prevents mdelv5 from being re-defined
if nsoln2 was 0.
mdelv5(ctrc)=min(mdelv4);
end
end
mdelv(ctr5,ctr6,nctr)=min(min(mdelv3,mdelv5)); %this picks the
smallest delta V out of all of the nrev runs.
end
end
end
figure(nctr);surf(mdelv(:,:,nctr));title(['Plot for window ',num2str(nctr),', target ',num2str(tgtctr),'.']);colorbar %a new figure is created for each
intercept window.
xlabel({['Position of the target using a window of ',num2str(t_win(nctr)),' minutes with a time step of ',num2str(t_tstep(nctr)),' minutes.'];['The beginning of the window was ',num2str(t_ep(nctr)),' minutes after day',num2str(satrec(1,tgtctr+1).epochdays),' in year',num2str(satrec(1,tgtctr+1).epochyr),'.']});
ylabel({['Position of the interceptor using a window of ',num2str(i_win),'minutes with a time step of ',num2str(i_tstep),' minutes.'];['The beginning of the window was ',num2str(i_ep),' minutes after day',num2str(satrec(1,1).epochdays),' in year',num2str(satrec(1,1).epochyr),'.']});
zlabel('Burn Magnitude (Canonical)');
end
%The code below allows to either have MATLAB pick the smallest burn
%value, or have the user input the desired burn.
44
disp(' ')
disp('After reviewing the plots, please indicate what mode to use to pick theburn/intercept combination.')
xdecide=input('User determines burn (1) or MATLAB picks first smallest Delta V (2) ');
if xdecide < 2
% this is for the user specified burn.
disp(' ')
disp('Using the data cursor')
disp('Pick the requested burn')
npick=input('From what window? ');
bcol=input('What is the x value? '); %this is the target value
brow=input('What is the y value? '); %this is the interceptor value
disp(' ')
disp(['The burn picked was ',num2str(mdelv(brow,bcol,npick)),' DU/TU'])
%this code allows the user to verify his pick
disp(['The burn occured at ',num2str(tsince(brow)),' minutes after day',num2str(satrec(1,1).epochdays),' in year',num2str(satrec(1,1).epochyr),'.'])
disp(['The intercept occrred at ',num2str(tsince2(bcol,npick)),' minutesafter day ',num2str(satrec(1,tgtctr+1).epochdays),' in year',num2str(satrec(1,tgtctr+1).epochyr),'.'])
disp(' ')
total_time=total_time+tsince2(bcol,npick)+deltaep*1440;
total_burn=total_burn+mdelv(brow,bcol,npick);
disp(['The total time since original epoch is ',num2str(total_time),'minutes.'])
disp(['The total burn is ',num2str(total_burn),' '])
for ctrc=1:5 %as all of the velocity required vectors were not saved from
above, piter must be re-run for the selected burn/intercept combination
if nrev(brow,bcol,npick)-3+ctrc < 0
continue
end
[nsoln,vra_xss(:,:,ctrc),v21]=piter(r_xss(:,brow),r_hst(:,bcol,npick),tdays(brow,bcol,npick),0,nrev(brow,bcol,npick)-3+ctrc);
[nsoln2,vrb_xss(:,:,ctrc),v22]=piter(r_xss(:,brow),r_hst(:,bcol,npick),tdays(brow,bcol,npick),1,nrev(brow,bcol,npick)-3+ctrc);
if nsoln<eps && nsoln2<eps
continue
end
for stu=1:nsoln
energy=0.5*(norm(vra_xss(:,stu,ctrc)))^2-1/norm((r_xss(:,brow)));
a=-1/(2*energy);
H=cross(r_xss(:,brow),vra_xss(:,stu,ctrc));
e=cross(vra_xss(:,stu,ctrc),H)-r_xss(:,brow)/norm(r_xss(:,brow));
rp=a*(1-e);
45
if rp-Re < 0 %this codes checks to see if the perigee hieight is
above earth or not
vra_xss(:,stu,ctrc)=[NaN;NaN;NaN];
end
end
for stu=1:nsoln2
energy=0.5*(norm(vrb_xss(:,stu)))^2-1/norm((r_xss(:,brow)));
a=-1/(2*energy);
H=cross(r_xss(:,brow),vrb_xss(:,stu));
e=cross(vrb_xss(:,stu),H)-r_xss(:,brow)/norm(r_xss(:,brow));
rp=a*(1-e);
if rp-Re < 0 %this codes checks to see if the perigee hieight is
above earth or not
vrb_xss(:,stu,ctrc)=[NaN;NaN;NaN];
end
end
end
for ctrc=1:size(vra_xss,3)
for ctrb=1:size(vra_xss(:,:,ctrc),2)
if norm(vra_xss(:,ctrb,ctrc)) < eps %if the code above skips a nrev
and then does a run later, MATLAB will put zeros in the skipped column
mdelva(ctrb,ctrc)=NaN;
continue
end
mdelva(ctrb,ctrc)=norm(vra_xss(:,ctrb,ctrc)-v_xss(:,brow));
end
end
for ctrc=1:size(vrb_xss,3)
for ctrb=1:size(vrb_xss(:,:,ctrc),2)
if norm(vrb_xss(:,ctrb,ctrc)) < eps
mdelvb(ctrb,ctrc)=NaN;
continue
end
mdelvb(ctrb,ctrc)=norm(vrb_xss(:,ctrb,ctrc)-v_xss(:,brow));
end
end
% Decide either a or b
deca=min(min(mdelva)); %this is used in determining which piter run has the
smalles delta V.
decb=min(min(mdelvb));
if deca<decb || isnan(decb)
46
if size(mdelva,1)<2 %in some cases mdelva will only have one row, and so
a slightly different way of finding the correct v is needed
[xc,xi]=min(mdelva);
ctrcnum=xi;
ctrbnum=1;
else
[xc,xi]=min(mdelva);
[xxc,xxi]=min(xc);
ctrbnum=xi(xxi);
ctrcnum=xxi;
end
%nv_xss=vra_xss(:,ctrbnum,ctrcnum); %this is the new velocity vector of
the internceptor at burn
sway=0;
else
if size(mdelvb,1)<2
[xc,xi]=min(mdelvb);
ctrcnum=xi;
ctrbnum=1;
else
[xc,xi]=min(mdelvb);
[xxc,xxi]=min(xc);
ctrbnum=xi(xxi);
ctrcnum=xxi;
end
%nv_xss=vrb_xss(:,ctrbnum,ctrcnum); %this is the new velocity vector of
the internceptor at burn
sway=1;
end
[nsoln,v11,vai_xss]=piter(r_xss(:,brow),r_hst(:,bcol,npick),tdays(brow,bcol,npick),sway,nrev(brow,bcol,npick)-3+ctrcnum);
nv_xss=vai_xss(:,ctrbnum); %this is the velocity of the interceptor at
intercept
else %this is the option for MATLAB determining the first smallest burn
for nctr=1:nwin
smdv(nctr)=min(min(mdelv(:,:,nctr)));
end
[bc,bi]=min(smdv);
disp(['The burn picked was ',num2str(bc),' DU/TU'])
npick=bi;
[bc,bi]=min(mdelv(:,:,npick));
[bbc,bbi]=min(bc);
brow=bi(bbi); %this is the row of mdelv
bcol=bbi; %this is the coloumn of mdelv
%the code below was copied from the option above
for ctrc=1:5
if nrev(brow,bcol,npick)-3+ctrc < 0
47
continue
end
[nsoln,vra_xss(:,:,ctrc),v21]=piter(r_xss(:,brow),r_hst(:,bcol,npick),tdays(brow,bcol,npick),0,nrev(brow,bcol,npick)-3+ctrc);
[nsoln2,vrb_xss(:,:,ctrc),v22]=piter(r_xss(:,brow),r_hst(:,bcol,npick),tdays(brow,bcol,npick),1,nrev(brow,bcol,npick)-3+ctrc);
if nsoln<eps && nsoln2<eps
continue
end
for stu=1:nsoln
energy=0.5*(norm(vra_xss(:,stu,ctrc)))^2-1/norm((r_xss(:,brow)));
a=-1/(2*energy);
H=cross(r_xss(:,brow),vra_xss(:,stu,ctrc));
e=cross(vra_xss(:,stu,ctrc),H)-r_xss(:,brow)/norm(r_xss(:,brow));
rp=a*(1-e);
if rp-Re < 0 %this codes checks to see if the perigee hieight is
above earth or not
vra_xss(:,stu,ctrc)=[NaN;NaN;NaN];
end
end
for stu=1:nsoln2
energy=0.5*(norm(vrb_xss(:,stu)))^2-1/norm((r_xss(:,brow)));
a=-1/(2*energy);
H=cross(r_xss(:,brow),vrb_xss(:,stu));
e=cross(vrb_xss(:,stu),H)-r_xss(:,brow)/norm(r_xss(:,brow));
rp=a*(1-e);
if rp-Re < 0 %this codes checks to see if the perigee hieight is
above earth or not
vrb_xss(:,stu,ctrc)=[NaN;NaN;NaN];
end
end
end
for ctrc=1:size(vra_xss,3)
for ctrb=1:size(vra_xss(:,:,ctrc),2)
if norm(vra_xss(:,ctrb,ctrc)) < eps
mdelva(ctrb,ctrc)=NaN;
continue
end
mdelva(ctrb,ctrc)=norm(vra_xss(:,ctrb,ctrc)-v_xss(:,brow));
end
end
for ctrc=1:size(vrb_xss,3)
for ctrb=1:size(vrb_xss(:,:,ctrc),2)
if norm(vrb_xss(:,ctrb,ctrc)) < eps
48
mdelvb(ctrb,ctrc)=NaN;
continue
end
mdelvb(ctrb,ctrc)=norm(vrb_xss(:,ctrb,ctrc)-v_xss(:,brow));
end
end
% Decide either a or b
deca=min(min(mdelva));
decb=min(min(mdelvb));
if deca<decb || isnan(decb)
if size(mdelva,1)<2
[xc,xi]=min(mdelva);
ctrcnum=xi;
ctrbnum=1;
else
[xc,xi]=min(mdelva);
[xxc,xxi]=min(xc);
ctrbnum=xi(xxi);
ctrcnum=xxi;
end
%nv_xss=vra_xss(:,ctrbnum,ctrcnum); %this is the new velocity vector of
the internceptor at burn
sway=0;
else
if size(mdelvb,1)<2
[xc,xi]=min(mdelvb);
ctrcnum=xi;
ctrbnum=1;
else
[xc,xi]=min(mdelvb);
[xxc,xxi]=min(xc);
ctrbnum=xi(xxi);
ctrcnum=xxi;
end
%nv_xss=vrb_xss(:,ctrbnum,ctrcnum); %this is the new velocity vector of
the internceptor at burn
sway=1;
end
[nsoln,v11,vai_xss]=piter(r_xss(:,brow),r_hst(:,bcol,npick),tdays(brow,bcol,npick),sway,nrev(brow,bcol,npick)-3+ctrcnum);
nv_xss=vai_xss(:,ctrbnum); %this is the velocity of the interceptor at intercept
disp(['The burn occured at ',num2str(tsince(brow)),' minutes after day ',num2str(satrec(1,1).epochdays),' in year ',num2str(satrec(1,1).epochyr),'.'])
49
disp(['The intercept occrred at ',num2str(tsince2(bcol,npick)),' minutes after day ',num2str(satrec(1,tgtctr+1).epochdays),' in year ',num2str(satrec(1,tgtctr+1).epochyr),'.'])
disp(' ')
total_time=total_time+tsince2(bcol,npick)+deltaep*1440;
total_burn=total_burn+mdelv(brow,bcol,npick);
disp(['The total time since original epoch is ',num2str(total_time),' minutes.'])
disp(['The total burn is ',num2str(total_burn),' '])
end
%nr_xss=r_xss(:,brow); %this is the position vector of the interceptor at burn. nr_xss=r_hst(:,bcol,npick); %this is the position vector of the interceptor
at interception
tint=tsince2(bcol,npick)+deltaep*1440; %this is the time in minutes since
interceptor TLE epoch at intercept.
tburn=tsince(brow); %this is the time in minutes since interceptor TLE epoch
at burn.
nr_xss=nr_xss*DU2Km; %converting units from Canonical to km/sec
nv_xss=nv_xss*VU2KmSec;
% I need to generate a new TLE based upon the r and v of the interceptor at
% intercept. This problem is not easily done. So I will create a TLE
% changing r and v into COEs and reusing the bstar and 1st/2nd derivative
% information from the original TLE. 
[p,a,ecc,incl,omega,argp,nu,m,arglat,truelon,lonper ] = rv2coe (nr_xss,nv_xss,398600.44); %the 6 COES associated with r and v.
incl=rad2deg(incl);omega=rad2deg(omega);argp=rad2deg(argp);m=rad2deg(m);
%TLEs use degrees
T=2*pi*sqrt(a^3/398600.44);
mm=86400/T; %mean motion in revs per day for the TLE
edays=tint/(24*60);
if edays+satrec(1,1).epochdays > 365 %this code determines the new epoch
year and epoch day
xyears=floor((edays+satrec(1,1).epochdays)/365);
newyear=satrec(1,1).epochyr+xyears;
newday=edays+satrec(1,1).epochdays-(xyears*365);
else
newyear=satrec(1,1).epochyr;
newday=edays+satrec(1,1).epochdays;
end
50
eyear=num2str(newyear,'%02.0f'); %this updates the year and day in the
interceptor TLE
eday=num2str(newday,'%012.8f');
longstr1(19:20)=eyear;
longstr1(21:32)=eday;
ninc=num2str(incl,'%08.4f'); %this updates the COEs in the TLE
nraan=num2str(omega,'%08.4f');
necc=num2str(ecc,'%07.0f');
nargp=num2str(argp,'%08.4f');
nma=num2str(m,'%08.4f');
nmm=num2str(mm,'%011.8f');
longstr2(9:16)=ninc;
longstr2(18:25)=nraan;
longstr2(27:33)=necc;
longstr2(35:42)=nargp;
longstr2(44:51)=nma;
longstr2(53:63)=nmm;
disp(' ')
disp(['the new epoch year is ',eyear, '.'])
disp(['the new epoch day is ',eday,'.'])
disp(['The new mean motion is ',nmm,'.'])
disp(' ')
[satrec(1,1), startmfe, stopmfe, deltamin] = twoline2rv(whichconst, longstr1, longstr2, typerun,typeinput); %this changes satrec(1,1) to the new interceptor orbit.
close all; clear smdv mdelv r_hst r_xss v_hst v_xss vra_xss vrb_xss %this clears out variables that are recreated in the next run, so old data is not used.
end %this is the end statment for multuple targets 51 function [nsoln,v1,v2] = piter(r1,r2,tdays,iway,nrev)
% piter Uses an iteration method to solve the two position vector and time
% of flight problem
%
% piter function has been translated from BASIC code written by Dr. William
Wiesel
% Translated by Captain Barry Witt August 2008
%
% [nsoln,v1,v2]=piter(r1,r2,tdays,iway,nrev)
%
% P iteration method, ref Bate, Mueller & White section 5.4
% ELLIPSES ONLY
% units are DU's, TU's, although input, t, is in days
% W. Wiesel, AFIT/ENY, November 1998
% BM&W referrs to Bate, Mueller & White, "Fundamental of Astrodynamics"
%
% NOTE:Piter does not work with colinear position vectors, despite a time
% difference between them.
%
% Outputs definitions
% nsoln - number of solutions found: 0 if it fails, 1 if it found 1
% solution (usually < 1 rev case), or 2 solutions found (for > 1 rev case)
% v1(i,isoln) - initial velocity vector, DU/TU, isoln = 1 or 2. This is
% the velocity required at launch, or burn. In the same frame as r1 and
% r2.
% v2(i,isoln) - final velocity vector, isoln = 1 or 2. This is the
% velocity of the inteceptor at the interception point
%
% Inputs definitions
% r1 - the position vector of the interceptor at the initial time, in DU,
% in ECEF frame
% r2 - the position vector at the requested point of interception, in DU,
% in ECEF frame
% tdays is elapsed time of flight, in days
% iway = 0 for "short way", else for "long way"
% nrev = number of whole revolutions, 0 if <1. Note: This is chosen
% independently of tof
xmu=1;
%calculate some preliminaries
nsoln=0;
TU2min=13.44686457;
%convert time of flight from days to TU
t = tdays * (1440 / TU2min);
rone=norm(r1);
rtwo=norm(r2);
cosdnu=dot(r1,r2)/(rone*rtwo);
52
%colinear case won't converge
if cosdnu>0.985
%disp('Position vectors are colinear in the same direction. Piter terminating.')
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN]; %i have chosen NaN as the
error return value
return
elseif cosdnu<-0.995
%disp('Position vectors are colinear in the opposite direction. Piter terminating')
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
%three auxillary quantities, BM&W 5.4-4
xk=rone*rtwo*(1-cosdnu);
xl=rone+rtwo;
xm=rone*rtwo*(1+cosdnu);
%parabolic limiting values for parameter p, BM&W 5.4-14, -15
pM=xk/(xl+sqrt(2*xm));
pp=xk/(xl-sqrt(2*xm));
%calculate discriminant for p limits, Denominator of BM&W 5.4-8
disc=4*xk*xk*xl*xl+4*xk*xk*(2*xm-xl*xl); %The denominator in the book has a
p, but this does not
if disc<0 %no elliptic solutions
%disp('There are no elliptic solutions. Piter terminating')
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
%BRANCH ON NUMBER OF EXPECTED SOLUTIONS
if nrev<eps
% less than one rev case: this is the case explicitly covered in BM&W.
% only one solution each way (short or long) is expected.
% opening strategy: bisection, 10X, start just inside parabolic limits
del=0.000001;
plow=pM+del;
phi=pp-del;
[toflo,dtdp]=tofp(plow,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,0);
if isnan(toflo) %this code stops piter if tofp returns NaN for tof
53
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
[tofhi,dtdp]=tofp(phi,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,0);
if isnan(tofhi)
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
for i=1:10 %bisect
p=(plow+phi)/2;
[tof,dtdp]=tofp(p,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,1); %decide
which
if isnan(tof)
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
if (t-toflo)*(t-tof)<0 %low half brackets root
phi=p;
tofhi=tof;
else %root is in upper half
plow=p;
toflo=tof;
end
end
%see if we are still pegged at one limit, and therefore never
%converged....
if (abs(p-pM-del)<0.0000000001) || (abs(p-pp+del)<0.0000000001)
%disp('There was no convergence on the solution. Piterterminating.')
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN]; %no convergence
return
end
%endgame: Newton-Rhapson loop
for i=1:10
imode=1;
[tof,dtdp]=tofp(p,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,imode);
if isnan(tof)
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
dt=tof-t;
dp=-dt/dtdp;
p=p+dp;
if abs(dp)<0.00000000001
break
end
end
%didn't converge...check value of tof
if (abs(t-tof))>0.00000001
54
%disp('The difference between time requested and tof found was too big, piter is stopping.')
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
%final processing: get v1 and v2
%calculate f & g functions, BM&W 5.2-3 thru 5.2-6
f=1-rtwo*(1-cosdnu)/p;
if iway<eps %short way
sindnu=sqrt(1-cosdnu*cosdnu);
else
sindnu=-sqrt(1-cosdnu*cosdnu);
end
g=rone*rtwo*sindnu/sqrt(xmu*p);
fdot=sqrt(xmu/p)*((1-cosdnu)/sindnu)*((1-cosdnu)/p-1/rone-1/rtwo);
gdot=1-rone*(1-cosdnu)/p;
for i=1:3
v1(i,1)=(r2(i)-f*r1(i))/g;
v2(i,1)=fdot*r1(i)+gdot*v1(i,1);
end
%declare success with one solution found
nsoln=1;
else
%Multirev case: two solutions expected. This case is not explicitly
%covered in BM&W. The modifications for the multirev case are called
%out in the code where they occur. Mostly in the evaluation of time of
%flight (tofp function). Experimentation shows two solutions are
%expected between the parabolic limits of pm and pp.
%get initial approximations to both solutions by scanning p interval
del=0.000001;
plow=pM+del;
phi=pp-del;
pguess(1)=0;
pguess(2)=0;
tprev=0;
nroots=0;
for i=1:200
p=plow*(200-i)/199+phi*(i-1)/199;
[tof,dtdp]=tofp(p,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,0);
if isnan(tof)
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
55
end
if tprev<eps
tprev=tof;
else
if (tof-t)*(tprev-t)<0 %we've bracked a root...which one?
if pguess(1)<eps
pguess(1)=p;
nroots=1;
else
pguess(2)=p;
nroots=2;
break
end
end
tprev=tof;
end
end
%both roots were not found
if nroots<eps
%disp('Unable to find both roots in piter.')
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
%loop on solutions found.
for ir=1:nroots %endgame: Newton-Rhapson loop
for i=1:10
imode=1;
[tof,dtdp]=tofp(pguess(ir),rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,imode);
if isnan(tof)
nsoln=0;v1=[NaN;NaN;NaN];v2=[NaN;NaN;NaN];
return
end
dt=tof-t;
dp=-dt/dtdp;
pguess(ir)=pguess(ir)+dp;
%check for negative p....abort
if pguess(ir)<0
break
end
if abs(dp)<0.0000000001
break
end
end
if pguess(ir)<0 %in Basic if this is true, the next ir loop goes through nsoln=nsoln+1; %if it skips an ir loop, the code neeeds nsoln to be advanced
56
else
% final processing: get v1 and v2. calculate f & g functions,
BM&W
% 5.2-3 thru 5.2-6
f=1-rtwo*(1-cosdnu)/pguess(ir);
if iway<eps %short way
sindnu=sqrt(1-cosdnu*cosdnu);
else
sindnu=-sqrt(1-cosdnu*cosdnu);
end
g=rone*rtwo*sindnu/sqrt(xmu*pguess(ir));
fdot=sqrt(xmu/pguess(ir))*((1-cosdnu)/sindnu)*((1- cosdnu)/pguess(ir)-1/rone-1/rtwo);
gdot=1-rone*(1-cosdnu)/pguess(ir);
nsoln=nsoln+1;
for i=1:3
v1(i,nsoln)=(r2(i)-f*r1(i))/g;
v2(i,nsoln)=fdot*r1(i)+gdot*v1(i,ir);
end
end
end
%end <1 rev/multirev if block
end
57
function [tof,dtofdp] = tofp(p,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,imode) %tofp function has been translated from BASIC code written by Dr. William Wiesel.
%Translated by Captain Barry Witt August 2008.
%
%This calculates time of flight, tof, from supposed value p value for p iteration
%method. In addition this calculates dt/dp if imode ??? 0.
%Original code written by W. Wiesel, AFIT/ENY, Nov 1998. "BM&W" refers to
%Bate, Mueller, and White, "Fundamentals of Astrodynamics", Dover.
%
%Outputs are tof,dtofdp. Inputs are p,rone,rtwo,cosdnu,xk,xl,xm,iway,nrev,imode.
%
%Outputs definitions
%tof - time of flight in TU units
%dtofdp - rate of change of tof w.r.t. p
%
%Inputs definitions
%p - semi-latus rectum in ??? units
%rone - magnitude of the position vector of first satellite in ???? units
%rtwo - maginitude of the position vector of the second satellite in ??? units
%cosdnu - cosine of the angle between the two position vectors.
%xk - constant defined in BM&W
%xl - constant defined in BM&W
%xm - constant defined in BM&W
%iway - short/long way determination. 0 is short, else is long.
%nrev - number of revolutions. 0 is less then one case covered in BM&W.
%Note: this is chosen independently of tof
%Else is developed in the code.
%imode - if this is equal to zero then code only calculates tof.
xmu=1;
a=xm*xk*p/((2*xm-xl*xl)*p*p+2*xk*xl*p-xk*xk); %calculate semimajor axis a, BM&W 5.4-8
if a<0
%disp('Accidental hyperbolic orbit')
tof=NaN;
dtofdp=NaN;
return
end
%calculate f, fdot, g: BM&W 5.2-3, 5.2-5, 5.2-4
f=1-rtwo*(1-cosdnu)/p;
if iway<eps %short way
sindnu=sqrt(1-cosdnu*cosdnu);
else
sindnu=-sqrt(1-cosdnu*cosdnu);
58
end
if p<0
%disp('p is negative.')
tof=NaN;
dtofdp=NaN;
return
end
g=rone*rtwo*sindnu/sqrt(xmu*p);
fdot=sqrt(xmu/p)*((1-cosdnu)/sindnu)*((1-cosdnu)/p-1/rone-1/rtwo); %there's a
half angle trig sub here.
%change in eccentric anomaly, BM&W 5.4-9, 5.4-10
cosdE=1-rone*(1-f)/a;
sindE=-rone*rtwo*fdot/sqrt(xmu*a);
dE=acos(cosdE);
if sindE<0
dE=2*pi-dE;
end
%ensure dE within 0-1 rev
if dE<0
dE=dE+2*pi;
end
if dE>2*pi
dE=dE-2*pi;
end
%add multiple revs to eccentric anomaly...required extension not in BM&W
dE=dE+2*pi*nrev;
%time of flight, BM&W 5.4-12
tof=g+sqrt(a*a*a/xmu)*(dE-sindE);
%is derivative not required?
if imode<eps
dtofdp=NaN;
return
end
%calculate dt/dp
%less-than-one rev derivative, BM&W 5.4-22
59
dtofdp=-g/(2*p)-1.5*a*(tof-sqrt(a*a*a)*2*pi*nrev-g)*((xk*xk+(2*xmxl* xl)*p*p)/(xm*xk*p*p))+sqrt(a*a*a/xmu)*2*xk*sindE/(p*(xk-xl*p));
%correction for multiple revs...nrev*d(DeltaE = 2 n pi)/da*da/dp
%NEEDS MU FIX
if nrev>0
dtofdp=dtofdp-3*nrev*pi*sqrt(a*a*a*a*a)*((xk*xk+(2*xmxl* xl)*p*p))/(xm*xk*p*p);
end
