function [out, experimentDir] = run_rvdlab_simulink_demo(showResults)
% Run the example through the RVDLAB external runtime and simulator.
arguments
    showResults (1,1) logical = true
end

folder = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(folder));
model = 'rvdlab_external_demo';
modelFile = fullfile(folder, [model '.slx']);
if bdIsLoaded(model) && ~strcmpi(get_param(model, 'FileName'), modelFile)
    error('RvdLab:ModelNameConflict', 'Another model named %s is open.', model);
end
load_system(modelFile);

python = getenv('RVDLAB_DEMO_PYTHON');
if isempty(python)
    python = 'python';
end

tempFolder = tempname;
mkdir(tempFolder);
readyFile = fullfile(tempFolder, 'ready.json');
logFile = fullfile(tempFolder, 'rvdlab.log');
steps = 17;
command = java.util.ArrayList();
for argument = {python, '-B', '-m', 'rvdlab', 'external', '--root', root, ...
                '--config', fullfile(folder, 'rail-demo.ini'), ...
                '--ready-file', readyFile, '--max-steps', num2str(steps), ...
                '--name', 'rvdlab-cwh-demo'}
    command.add(java.lang.String(argument{1}));
end
builder = java.lang.ProcessBuilder(command);
builder.directory(java.io.File(root));
builder.redirectErrorStream(true);
builder.redirectOutput(java.io.File(logFile));
pythonPath = [fullfile(root, 'tui') pathsep fullfile(root, 'node')];
existingPath = getenv('PYTHONPATH');
if ~isempty(existingPath)
    pythonPath = [pythonPath pathsep existingPath];
end
environment = builder.environment();
environment.put('PYTHONPATH', pythonPath);
try
    process = builder.start();
catch exception
    removeTemporaryFiles(tempFolder, readyFile, logFile);
    rethrow(exception)
end
cleanup = onCleanup(@() cleanupDemo(process, tempFolder, readyFile, logFile));

started = tic;
while ~isfile(readyFile) && process.isAlive() && toc(started) < 15
    pause(0.05);
end
if ~isfile(readyFile)
    error('RvdLab:StartupFailed', 'Could not start the external runtime.\n%s', fileread(logFile));
end
endpoint = jsondecode(fileread(readyFile));
assert(strcmp(endpoint.host, '127.0.0.1'), 'The example requires a loopback endpoint.');
experimentDir = string(endpoint.experiment_dir);

in = Simulink.SimulationInput(model);
in = in.setVariable('external_port', double(endpoint.port), 'Workspace', model);
in = in.setVariable('external_token', uint8(endpoint.token), 'Workspace', model);
in = in.setVariable('demo_period', double(endpoint.period_s), 'Workspace', model);
in = in.setVariable('external_steps', steps, 'Workspace', model);
in = in.setModelParameter('StopTime', num2str((steps - 1)*endpoint.period_s), ...
    'SimulationMode', 'normal', 'EnablePacing', 'off');
out = sim(in);

if ~waitForExit(process, 5) || process.exitValue() ~= 0
    error('RvdLab:RuntimeFailed', 'The external runtime did not finish normally.\n%s', fileread(logFile));
end

summary = jsondecode(fileread(fullfile(experimentDir, 'summary.json')));
assert(strcmp(summary.outcome, 'COMPLETE') && strcmp(summary.reason, 'MAX_STEPS'), ...
    'RVDLAB did not record a complete experiment.');
fprintf('Experiment record: %s\n', experimentDir);

if showResults
    if usejava('desktop')
        open_system(model);
    end
    plotResults(out);
end
clear cleanup
end

function stopped = waitForExit(process, timeout)
started = tic;
while process.isAlive() && toc(started) < timeout
    pause(0.05);
end
stopped = ~process.isAlive();
end

function cleanupDemo(process, folder, readyFile, logFile)
stopRuntime(process);
removeTemporaryFiles(folder, readyFile, logFile);
end

function stopRuntime(process)
if ~waitForExit(process, 3)
    process.destroy();
    if ~waitForExit(process, 1)
        process.destroyForcibly();
        waitForExit(process, 1);
    end
end
end

function removeTemporaryFiles(folder, readyFile, logFile)
for file = {readyFile, logFile}
    if isfile(file{1})
        delete(file{1});
    end
end
if isfolder(folder) && numel(dir(folder)) == 2
    rmdir(folder);
end
end

function plotResults(out)
measured = out.hill_measured_m;
reference = out.hill_reference;
measuredValues = sampleRows(measured);
referenceValues = sampleRows(reference);
figure('Name', 'RVDLAB Simulink demo', 'NumberTitle', 'off');
layout = tiledlayout(2, 1, 'TileSpacing', 'compact');
axesNames = ["X", "Y"];
for axisIndex = 1:2
    ax = nexttile(layout);
    plot(ax, measured.Time, measuredValues(:,axisIndex), 'o-', 'LineWidth', 1.2);
    hold(ax, 'on');
    stairs(ax, reference.Time, referenceValues(:,axisIndex), '--', 'LineWidth', 1.2);
    ylabel(ax, "Hill " + axesNames(axisIndex) + " (m)");
    grid(ax, 'on');
    if axisIndex == 1
        legend(ax, 'Measured position', 'Position reference', 'Location', 'best');
    end
end
xlabel(layout, 'Time (s)');
end

function rows = sampleRows(series)
sampleCount = numel(series.Time);
if series.IsTimeFirst
    rows = reshape(series.Data, sampleCount, []);
else
    rows = reshape(series.Data, [], sampleCount).';
end
end
