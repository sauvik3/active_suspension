%% Usage of CRG_FILTER
% Introducing the usage of crg_filter.
% Examples are included.
% The file comments are optimized for the matlab publishing makro.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               test
% file name:             crg_test_filter.m
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
% * load crg-file
% * filter image
% * display result
%

% DEFAULT SETTINGS
% clear enviroment
clear all;
close all;

% Set plot area
scrpos = get(0,'ScreenSize');
figpos(4) = min(scrpos(3)/sqrt(2), scrpos(4)) * 0.8;    % heigth
figpos(3) = figpos(4)*sqrt(2);                          % width
figpos(1) = scrpos(1) + scrpos(3)*0.9 - figpos(3);      % left

%% Test1 ( mean filter: smooth road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'mean', [10 10], [1 20]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test1.1 ( mean filter: smooth road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'mean', [20 20], [1 15]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test1.2 (mean filter: real -> smooth road)

data = crg_read('../crg-bin/belgian_block.crg');

dat = crg_filter(data, [400 900], [50 300], 'mean', [10 10], [1 5]);

figure('Position', figpos);
crg_plot_road_xyz_map(dat);

%% Test2 ( gauss filter: smooth road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'gauss', [10 10], [1 30]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test2.1 ( gauss filter: smooth road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'gauss', [20 20], [1 40]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test2.2 (gauss filter: real -> smooth road)

data = crg_read('../crg-bin/belgian_block.crg');

dat = crg_filter(data, [400 900], [50 300], 'gauss', [10 10], [1 5]);

figure('Position', figpos);
crg_plot_road_xyz_map(dat);

%% Test3 ( laplace: bumpy road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'laplace', [3 3], [1 6]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test3.1 ( laplace: bumpy road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'laplace', [5 5], [1 25]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test3.2 (laplace: real -> bumpy road)

data = crg_read('../crg-bin/belgian_block.crg');

dat = crg_filter(data, [400 900], [50 300], 'laplace', [10 10], [1 20]);

figure('Position', figpos);
crg_plot_road_xyz_map(dat);

%% Test4 ( sobel: bumpy road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'sobel', [3 3], [1 10]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test4.1 ( sobel: bumpy road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], 'sobel', [5 5], [1 25]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test4.2 (sobel: real -> bumpy road)

data = crg_read('../crg-bin/belgian_block.crg');

dat = crg_filter(data, [400 900], [50 300], 'sobel', [10 10], [1 30]);

figure('Position', figpos);
crg_plot_road_xyz_map(dat);

%% Test5 ( 2diff: bumpy road )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], '2diff', [3 3], [1 5]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test5.1 ( 2diff )

data = crg_read('demo1.crg');

data = crg_filter(data, [200 700], [1 201], '2diff', [10 10], [1 55]);

figure('Position', figpos);
crg_plot_road_xyz_map(data);

%% Test5.2 ( 2diff )

data = crg_read('../crg-bin/belgian_block.crg');

dat = crg_filter(data, [400 900], [50 300], '2diff', [10 10], [1 30]);

figure('Position', figpos);
crg_plot_road_xyz_map(dat);
