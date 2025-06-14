%% Usage of CRG_PEAKFINDER
% Introducing the usage of crg_peakfinder.
% Examples are included.
% The file comments are optimized for the matlab publishing makro.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               test
% file name:             crg_test_peakfinder.m
% author:                ASAM e.V.
%
%
% C by ASAM e.V., 2020
% Any use is limited to the scope described in the license terms.
% The license terms can be viewed at www.asam.net/license
%
% More Information on ASAM OpenCRG can be found here:
% https://www.asam.net/standards/detail/opencrg/
%
% *****************************************************************

%% Test proceedings
%
% * Load demo file
% * add peaks
% * find peaks
% * display result
%

% DEFAULT SETTINGS
% clear enviroment
clear all;
close all;

%% Test1

data = crg_read('demo8.crg');

% add peaks
data.z(200:210, 25:35) = 0.5;      % 10x10 + 0.05
data.z(300:305, 50:55) = 0.2;      % 5x5   + 0.09
data.z(800, 50) = -0.5;            % 1x1   - 0.5

iu = [1 1000];
iv = [1 200];

[pindex, pij] = crg_peakfinder( data, [], [], 0.5, 10);

crg_show_peaks(data, pij, iu, iv, [], []);

%% Test1.1 (sub selection)

data = crg_read('demo8.crg');

% add peaks
data.z(200:210, 25:35) = 0.5;      % 10x10 + 0.05
data.z(300:305, 50:55) = 0.2;      % 5x5   + 0.09
data.z(800, 50) = -0.5;            % 1x1   - 0.5

iu = [1 400];
iv = [1 100];

[pindex, pij] = crg_peakfinder( data, iu, iv, 0.5, 10);

crg_show_peaks(data, pij, iu, iv);

%% Test1.2 (sub selection)

data = crg_read('demo8.crg');

% add peaks
data.z(200:210, 25:35) = 0.5;      % 10x10 + 0.05
data.z(300:305, 50:55) = 0.2;      % 5x5   + 0.09
data.z(800, 50) = -0.5;            % 1x1   - 0.5

iu = [1 400];
iv = [1 100];

[pindex, pij] = crg_peakfinder( data, iu, iv, 0.5, 10);

crg_show_peaks(data, pij, [], [], iu, iv);

%% Test2 (real)

dat1 = crg_read('../crg-bin/country_road.crg');

% add peaks
dat1.z(200:210, 25:35) = 0.5;      % 10x10 + 0.05
dat1.z(300:305, 50:55) = 0.2;      % 5x5   + 0.09
dat1.z(800, 50) = 0.5;             % 1x1   - 0.03

iu = [1 1000];
iv = [1 150];

[pindex, pij] = crg_peakfinder( dat1, iu, iv, 0.5, 10);

crg_show_peaks(dat1, pij, iu, iv, [1 1000]);
