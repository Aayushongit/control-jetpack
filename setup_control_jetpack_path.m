function setup_control_jetpack_path(projectRoot)
if nargin < 1
    projectRoot = fileparts(mfilename('fullpath'));
end

matlabDir = fullfile(projectRoot, 'matlab');
srcDir = fullfile(matlabDir, 'src');
testsDir = fullfile(matlabDir, 'tests');

addpath(matlabDir);
addpath(srcDir);
addpath(testsDir);
rehash;
end
