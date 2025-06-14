function [data] = crg_separate_sb(data, swlen, bwlen)
% CRG_SEPARATE_SB Separate slope and banking.
%   [DATA] = CRG_SEPARATE_SB(DATA, SWLEN, BWLEN) finds and filters slope
%   and banking in OpenCRG road data. It separates the filtered result from
%   the road data - leaving the total elevation information unchanged.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO
%   SWLEN   slope filter window length [m]
%           >  0: separate filtered slope from elevation grid
%           =  0: don't apply any filtering, don't separate any slope
%           = -1: don't apply any filtering, just separate local slope for
%                 each cross cut mean elevation at v=0
%           = -2: separate constant slope between begin and end cross
%                 cut mean elevation at v=0.
%   BWLEN   banking filter window length [m]
%           >  0: separate filtered banking from elevation grid
%           =  0: don't apply any filtering, don't separate any banking
%           = -1: don't apply any filtering, just separate local banking
%                 for each cross cut
%           = -2: find and separate constant mean banking
%
%   Outputs:
%   DATA    struct array as defined in CRG_INTRO
%
%   Examples:
%   crg = crg_separate_sb(crg, 20, 20)
%       separates banking and slope filtered with 20m window length
%   crg = crg_separate_sb(crg, 0, 0)
%       returns a crg with slope and banking merged into the elevation grid

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_separate_sb.m 
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

%% check if already successfully checked

if ~isfield(data, 'ok')
    data = crg_check(data);
    if ~isfield(data, 'ok')
        error('CRG:checkError', 'check of DATA was not completely successful')
    end
end

%% evaluate DATA.z size

[nu] = size(data.z, 1);

%% build full v-vector

if isfield(data.head, 'vinc')
    vmin = data.head.vmin;
    vmax = data.head.vmax;
    vinc = data.head.vinc;
    v = vmin:vinc:vmax;
else
    v = double(data.v);
end

%% build full rz-vector

if isfield(data, 'rz')
    rz = data.rz;
else
    rz = linspace(data.head.zbeg, data.head.zend, nu);
end

%% get old banking

if isfield(data, 'b')
    if length(data.b) == 1
        bold = data.head.bbeg*ones(1, nu);
    else
        bold = double(data.b);
    end
else
    bold = zeros(1, nu);
end

%% get individual z(v=0) and b for each cross cut

z = zeros(1,nu);
b = zeros(1,nu);

for i = 1:nu
    pz = data.z(i,:) + rz(i) + bold(i).*v;

    valid = ~isnan(pz);

    % linear regression on cross cut: least squares fit
    zreg = [v(valid)' ones(sum(valid),1)]\pz(valid)';
    z(i) = zreg(2);
    b(i) = zreg(1);
end

%% find and apply slope

if swlen > 0
    w = round(swlen/data.head.uinc/2);
    if w < nu
        opts = struct;
        opts.window  = 'hann';
        opts.reflect = 'point';
        zf = smooth_firfilt(z, w, opts);
        swlen = 1;
    else
        swlen = -2;
    end
end

switch swlen
    case  1 % filtered slope
        % see above
    case  0 % no slope
        zf = mean(z) * ones(1, nu);
    case -1 % un-filtered slope
        zf = z;
    case -2 % constant slope
        zf = linspace(z(1), z(end), nu);
    otherwise
        error('CRG:separate_sb', 'illegal SWLEN value')
end

data = crg_s2z(data, zf);

%% find and apply banking

if bwlen > 0
    w = round(bwlen/data.head.uinc/2);
    if w < nu
        opts = struct;
        opts.window  = 'hann';
        opts.reflect = 'point';
        bf = smooth_firfilt(b, w, opts);
        bwlen = 1;
    else
        bwlen = -2;
    end
end

switch bwlen
    case  1 % filtered banking
        % see above
    case  0 % no banking
        bf = zeros(1, nu);
    case -1 % un-filtered banking
        bf = b;
    case -2 % constant banking
        bf = mean(b) * ones(1, nu);
    otherwise
        error('CRG:separate_sb', 'illegal BWLEN value')
end

data = crg_b2z(data, bf);

end
