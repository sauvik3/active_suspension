function [data] = crg_show_peaks(data, pindex, su, sv, iu, iv)
% CRG_SHOW_PEAKS Visualize peaks.
%   DATA = CRG_SHOW_PEAKS(DATA, PINDEX, SU, SV, IU, IV) visualizes peaks in
%   OpenCRG data. Peaks are usually identified via CRG_PEAKFINDER. 
%   The plots and the display of peaks in these plots can be
%   limited to a selected area of the grid.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO
%   PINDEX  peak index in data.z
%   SU      u-index of peak-search selection (default: IU)
%   SV      v-ndex of peak-search selection (default: IV)
%   IU      U index for plot selection (default: full CRG)
%           IU(1): longitudinal start index
%           IU(2): longitudinal end index
%   IV      V index for plot selection (default: full CRG)
%           IV(1): lateral start index
%           IV(2): lateral end index
%
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   data = crg_show_peaks(data, pindex)
%       Visualizes the full CRG.
%   data = crg_show_peaks(data, pindex, [1000 2000], [10 20])
%       Visualizes the selected part of the CRG.
%   See also CRG_INTRO, CRG_PEAKFINDER.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_show_peaks.m 
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

%% first check, fix and complement DATA
[nu nv] = size(data.z);

if nargin < 6 || isempty(iv),   iv =  [1 nv]; end
if nargin < 5 || isempty(iu),   iu =  [1 nu]; end
if nargin < 4 || isempty(sv),   sv =  iv; end
if nargin < 3 || isempty(su),   su =  iu; end

if length(sv) == 1, sv = [1 sv]; end
if length(su) == 1, su = [1 su]; end
if length(iv) == 1, iv = [1 iv]; end
if length(iu) == 1, iu = [1 iu]; end

if su > iu
    error('CRG:showError', 'index iu less than visualized area of peaks uv');
end

if sv > iv
    error('CRG:showError', 'index iv less than visualized area of peaks sv');
end

%% check if already succesfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of DATA was not completely successful')
    end
end

%% check and complement optional arguments


if iu(1)<1 || iu(1) >= iu(2) || iu(2) > nu
    error('CRG:showError', 'illegal IU index values iu=[%d %d] with nu=%d', iu(1), iu(2), nu);
end


if iv(1)<1 || iv(1) >= iv(2) || iv(2) > nv
    error('CRG:showError', 'illegal IV index values iv=[%d %d] with nv=%d', iv(1), iv(2), nv);
end

%% define figure frame

if ~isfield(data, 'fopt') || ~isfield(data.fopt, 'tit')
    data.fopt.tit = 'CRG peak visualizer';
    data          = crg_figure(data);
    data.fopt     = rmfield(data.fopt, 'tit');
else
    data = crg_figure(data);
end

%% reference line XY overview map

subplot(2,2,1)
data = crg_plot_refline_xy_overview_map(data, iu);
set(    gca             , 'ButtonDownFcn','copy_ax2fig')
set(get(gca, 'Children'), 'ButtonDownFcn','copy_ax2fig')

%% elevation grid XYZ perspective map

subplot(2,2,2)
hold on
data = crg_plot_road_xyz_map(data, iu, iv);
set(    gca             , 'ButtonDownFcn','copy_ax2fig')
set(get(gca, 'Children'), 'ButtonDownFcn','copy_ax2fig')
data = crg_vis_peaks(data, pindex, 0, su, sv);
hold off

%% elevation grid UVZ orthographic map

subplot(2,2,[3 4])
hold on
data = crg_plot_road_uvz_map(data, iu, iv);
set(    gca             , 'ButtonDownFcn','copy_ax2fig')
set(get(gca, 'Children'), 'ButtonDownFcn','copy_ax2fig')
data = crg_vis_peaks(data, pindex, 1, su, sv);
hold off

end % function crg_show_peaks

function [data] = crg_vis_peaks(data, pindex, uv, su, sv)
% CRG_VIS_PEAKS Visualize peak.
%   DATA = CRG_VIS_PEAKS(DATA, PINDEX, UV, SU, SV) visualizes peaks.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO
%   PINDEX  peak index in data.z
%   UV      plot uvz(1) or xyz(0)
%   SU      u-index of peak-search selection (default: full CRG)
%   SV      v-ndex of peak-search selection (default: full CRG)
%
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   data = crg_vis_peaks(data, pindex)
%       Visualize peaks on full CRG.
%   See also CRG_SHOW_PEAKS, INTRO.

%% build uv-grid

ubeg = data.head.ubeg;
uend = data.head.uend;
uinc = data.head.uinc;
u = ubeg:uinc:uend;

if isfield(data.head, 'vinc')
    vmin = data.head.vmin;
    vmax = data.head.vmax;
    vinc = data.head.vinc;
    v = vmin:vinc:vmax;
else
    v = data.v;
end

%% visualize data

su = u(su)';
sv = v(sv)';

u = u(pindex(:,1))';
v = v(pindex(:,2))';

if uv   % plot uvz
    pz = crg_eval_uv2z(data, [u,v]);

    mz = max(max(data.z));
    if isfield(data, 's'), mz = mz + max(data.s); end
    if isfield(data, 'b'), mz = mz + max(data.b); end

    plot3(u, v, pz, 'mo', ...
                    'LineWidth', 2, ...
                    'MarkerEdgeColor','k', ...
                    'MarkerFaceColor',[1 1 1], ...
                    'MarkerSize', 10);


    plot3([su; su(2); su(1); su(1)], [sv(1); sv(1); sv(2); sv(2); sv(1)], [mz mz mz mz mz], '--m', ...
                    'LineWidth', 2);
else    % plot xyz
    pz = crg_eval_uv2z(data, [u,v]);
    puv = crg_eval_uv2xy(data, [u,v]);

    plot3(puv(:,1), puv(:,2), pz, 'mo', ...
                'LineWidth', 2, ...
                'MarkerEdgeColor','k', ...
                'MarkerFaceColor',[1 1 1], ...
                'MarkerSize', 10);

end

end % function crg_vis_peaks
