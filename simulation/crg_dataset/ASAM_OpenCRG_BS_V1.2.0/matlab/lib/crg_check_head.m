function [data] = crg_check_head(data)
% CRG_CHECK_HEAD Check OpenCRG road parameters data.
%   [DATA] = CRG_CHECK_HEAD(DATA) checks OpenCRG road parameters data for
%   consistent definitions and values.
%
%   Inputs:
%   DATA    struct array as defined in CRG_INTRO.
%
%   Outputs:
%   DATA    input DATA with checked road parameters data.
%
%   Examples:
%   data = crg_check_head(data)
%       Checks CRG road parameters data.
%
%   See also CRG_CHECK, CRG_INTRO.

% *****************************************************************
% ASAM OpenCRG Matlab API
%
% OpenCRG version:           1.2.0
%
% package:               lib
% file name:             crg_check_head.m 
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

%% remove ok flag, initialize error/warning counter

if isfield(data, 'ok')
    data = rmfield(data, 'ok');
end
ierr = 0;

%% check for head field

if ~isfield(data, 'head')
        data.head = struct;
end

%%  keep only what we know and want

head = struct;

if isfield(data.head, 'ubeg'), head.ubeg = data.head.ubeg; end
if isfield(data.head, 'uend'), head.uend = data.head.uend; end
if isfield(data.head, 'uinc'), head.uinc = data.head.uinc; end
if isfield(data.head, 'vmin'), head.vmin = data.head.vmin; end
if isfield(data.head, 'vmax'), head.vmax = data.head.vmax; end
if isfield(data.head, 'vinc'), head.vinc = data.head.vinc; end
if isfield(data.head, 'sbeg'), head.sbeg = data.head.sbeg; end
if isfield(data.head, 'send'), head.send = data.head.send; end
if isfield(data.head, 'bbeg'), head.bbeg = data.head.bbeg; end
if isfield(data.head, 'bend'), head.bend = data.head.bend; end
if isfield(data.head, 'xbeg'), head.xbeg = data.head.xbeg; end
if isfield(data.head, 'ybeg'), head.ybeg = data.head.ybeg; end
if isfield(data.head, 'xend'), head.xend = data.head.xend; end
if isfield(data.head, 'yend'), head.yend = data.head.yend; end
if isfield(data.head, 'xoff'), head.xoff = data.head.xoff; end
if isfield(data.head, 'yoff'), head.yoff = data.head.yoff; end
if isfield(data.head, 'pbeg'), head.pbeg = data.head.pbeg; end
if isfield(data.head, 'pend'), head.pend = data.head.pend; end
if isfield(data.head, 'poff'), head.poff = data.head.poff; end
if isfield(data.head, 'zbeg'), head.zbeg = data.head.zbeg; end
if isfield(data.head, 'zend'), head.zend = data.head.zend; end
if isfield(data.head, 'zoff'), head.zoff = data.head.zoff; end
if isfield(data.head, 'ebeg'), head.ebeg = data.head.ebeg; end
if isfield(data.head, 'nbeg'), head.nbeg = data.head.nbeg; end
if isfield(data.head, 'eend'), head.eend = data.head.eend; end
if isfield(data.head, 'nend'), head.nend = data.head.nend; end
if isfield(data.head, 'abeg'), head.abeg = data.head.abeg; end
if isfield(data.head, 'aend'), head.aend = data.head.aend; end

data.head = head;

%% some local variables

crgeps = data.opts.ceps;
crgtol = data.opts.ctol;
mininc = data.opts.cinc*(1-crgeps);
midinc = data.opts.cinc;

%% check singular value ranges

% CRG curved coordinate system
if isfield(data.head, 'uinc')
    if data.head.uinc < mininc
        error('CRG:checkError', 'illegal header data "reference_line_increment": %d', data.head.uinc)
    end
    if abs(round(data.head.uinc/midinc)*midinc - data.head.uinc) > crgeps*max(midinc, data.head.uinc)
        warning('CRG:checkWarning', 'header data "reference_line_increment": %d not a multiple of minimal CRG increment %d', data.head.uinc, midinc)
        ierr = 1;
    end
end
if isfield(data.head, 'ubeg')
    if abs(round(data.head.ubeg/midinc)*midinc - data.head.ubeg) > crgeps*max(midinc, abs(data.head.ubeg))
        warning('CRG:checkWarning', 'header data "reference_line_start_u": %d not a multiple of minimal CRG increment %d', data.head.ubeg, midinc)
        ierr = 1;
    end
end
if isfield(data.head, 'uend')
    if abs(round(data.head.uend/midinc)*midinc - data.head.uend) > crgeps*max(midinc, abs(data.head.uend))
        warning('CRG:checkWarning', 'header data "reference_line_end_u": %d not a multiple of minimal CRG increment %d', data.head.uend, midinc)
        ierr = 1;
    end
end
if isfield(data.head, 'vinc')
    if data.head.vinc < mininc
        error('CRG:checkError', 'illegal header data "reference_line_v_increment": %d', data.head.vinc)
    end
    if abs(round(data.head.vinc/midinc)*midinc - data.head.vinc) > crgeps*max(midinc, data.head.vinc)
        warning('CRG:checkWarning', 'header data "reference_line_v_increment": %d not a multiple of minimal CRG increment %d', data.head.vinc, midinc)
        ierr = 1;
    end
end
if isfield(data.head, 'vmin')
    if abs(round(data.head.vmin/midinc)*midinc - data.head.vmin) > crgeps*max(midinc, abs(data.head.vmin))
        warning('CRG:checkWarning', 'header data "long_section_v_right": %d not a multiple of minimal CRG increment %d', data.head.vmin, midinc)
        ierr = 1;
    end
end
if isfield(data.head, 'vmax')
    if abs(round(data.head.vmax/midinc)*midinc - data.head.vmax) > crgeps*max(midinc, abs(data.head.vmax))
        warning('CRG:checkWarning', 'header data "long_section_v_left": %d not a multiple of minimal CRG increment %d', data.head.vmax, midinc)
        ierr = 1;
    end
end
if isfield(data.head, 'sbeg')
    if abs(data.head.sbeg) > 1
        error('CRG:checkError', 'illegal header data "reference_line_start_s": %d', data.head.sbeg)
    end
end
if isfield(data.head, 'send')
    if abs(data.head.send) > 1
        error('CRG:checkError', 'illegal header data "reference_line_end_s": %d', data.head.send)
    end
end
if isfield(data.head, 'bbeg')
    if abs(data.head.bbeg) > 1
        error('CRG:checkError', 'illegal header data "reference_line_start_c": %d', data.head.bbeg)
    end
end
if isfield(data.head, 'bend')
    if abs(data.head.bend) > 1
        error('CRG:checkError', 'illegal header data "reference_line_end_c": %d', data.head.bend)
    end
end

% local rectangular coordinate system:
if isfield(data.head, 'pbeg')
    if abs(data.head.pbeg) > pi*(1+crgeps)
        warning('CRG:checkWarning', 'header reference_line_start_phi=%d outside range', data.head.pbeg)
        ierr = 1;
    end
end
if isfield(data.head, 'pend')
    if abs(data.head.pend) > pi*(1+crgeps)
        warning('CRG:checkWarning', 'header reference_line_end_phi=%d outside range', data.head.pend)
        ierr = 1;
    end
end
if isfield(data.head, 'poff')
    if abs(data.head.poff) > pi*(1+crgeps)
        warning('CRG:checkWarning', 'header reference_line_offset_phi=%d outside range', data.head.poff)
        ierr = 1;
    end
end

% WGS84 world geodetic system:
if isfield(data.head, 'ebeg')
    if abs(data.head.ebeg) > 180*(1+crgeps)
        error('CRG:checkError', 'illegal header data "reference_line_start_lon": %d', data.head.ebeg)
    end
end
if isfield(data.head, 'eend')
    if abs(data.head.eend) > 180*(1+crgeps)
        error('CRG:checkError', 'illegal header data "reference_line_end_lon": %d', data.head.eend)
    end
end
if isfield(data.head, 'nbeg')
    if abs(data.head.nbeg) > 90*(1+crgeps)
        error('CRG:checkError', 'illegal header data "reference_line_start_lat": %d', data.head.nbeg)
    end
end
if isfield(data.head, 'nend')
    if abs(data.head.nend) > 90*(1+crgeps)
        error('CRG:checkError', 'illegal header data "reference_line_end_lat": %d', data.head.nend)
    end
end

%% check for existing start if end is defined

% CRG curved coordinate system
if isfield(data.head, 'uend') && ~isfield(data.head, 'ubeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_u"')
end
if isfield(data.head, 'vmax') && ~isfield(data.head, 'vmin')
    error('CRG:checkError', 'missing definition of header data "reference_line_v_right"')
end
if isfield(data.head, 'send') && ~isfield(data.head, 'sbeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_s"')
end
if isfield(data.head, 'bend') && ~isfield(data.head, 'bbeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_c"')
end

% local rectangular coordinate system
if isfield(data.head, 'xend') && ~isfield(data.head, 'xbeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_x"')
end
if isfield(data.head, 'yend') && ~isfield(data.head, 'ybeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_y"')
end
if isfield(data.head, 'zend') && ~isfield(data.head, 'zbeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_z"')
end
if isfield(data.head, 'pend') && ~isfield(data.head, 'pbeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_phi"')
end

% WGS84 world geodetic system
if isfield(data.head, 'eend') && ~isfield(data.head, 'ebeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_lon"')
end
if isfield(data.head, 'nend') && ~isfield(data.head, 'nbeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_lat"')
end
if isfield(data.head, 'aend') && ~isfield(data.head, 'abeg')
    error('CRG:checkError', 'missing definition of header data "reference_line_start_alt"')
end

%% check for pairs

% local rectangular coordinate system
if xor(isfield(data.head, 'xbeg'), isfield(data.head, 'ybeg'))
    error('CRG:checkError', 'one missing definition of header data "reference_line_start_x/y"')
end
if xor(isfield(data.head, 'xend'), isfield(data.head, 'yend'))
    error('CRG:checkError', 'one missing definition of header data "reference_line_end_x/y"')
end
if xor(isfield(data.head, 'xoff'), isfield(data.head, 'yoff'))
    error('CRG:checkError', 'one missing definition of header data "reference_line_offset_x/y"')
end

% WGS84 world geodetic system
if xor(isfield(data.head, 'ebeg'), isfield(data.head, 'nbeg'))
    error('CRG:checkError', 'one missing definition of header data "reference_line_start_lon/lat"')
end
if xor(isfield(data.head, 'eend'), isfield(data.head, 'nend'))
    error('CRG:checkError', 'one missing definition of header data "reference_line_end_lon/lat"')
end

%% check pair ranges

% CRG curved coordinate system
if isfield(data.head, 'ubeg') && isfield(data.head, 'uend')
    if data.head.ubeg >= data.head.uend
        error('CRG:checkError', 'inconsistent definition of header data "reference_line_start/end_u"')
    end
end
if isfield(data.head, 'vmin') && isfield(data.head, 'vmax')
    if data.head.vmin >= data.head.vmax
        error('CRG:checkError', 'inconsistent definition of header data "reference_line_v_right/left"')
    end
end

%% check for consistent start/end distance & altitude definitions
[data] = crg_check_wgs84(data);

%% set ok-flag

if ierr == 0
    data.ok = 0;
end

end
