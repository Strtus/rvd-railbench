import subprocess

import pytest
from textual.widgets import Input, Select, Static

from rvdlab.app import RvdLabApp
from rvdlab.comparison import Comparison
from rvdlab.plot import FILTERED, HIGHLIGHT, NOMINAL, RAW, PlotView, Trace, braille_plot
from rvdlab.project import find_build

from conftest import ROOT
from test_app import text, wait_for


def test_overlays_expand_bounds_and_preserve_actual_marker():
    plot = braille_plot([0, 1], [0, 1], 60, 15, markers=[(1, 1, '●')],
                        highlights=[(1, 1)], overlays=[Trace([0, 10], [2, 2], NOMINAL, True)])
    assert '10.0' in plot.plain
    assert '●' in plot.plain
    assert any(span.style == NOMINAL for span in plot.spans)
    assert any(span.style == HIGHLIGHT and '●' in plot.plain[span.start:span.end] for span in plot.spans)


@pytest.mark.skipif(find_build(ROOT).rail_replay is None, reason='rail_replay is not built')
async def test_simulation_comparison_and_local_prediction(tmp_path, profiles):
    app = RvdLabApp(root=ROOT, experiments=tmp_path / 'experiments', profiles=profiles)
    async with app.run_test(size=(160, 54)) as pilot:
        await pilot.pause()
        await pilot.press('3', 'enter')
        await pilot.pause()
        screen = app.screen
        screen.query_one('#clock', Select).value = 'fast'
        screen.query_one('#max-steps', Input).value = '150'
        screen.start()
        await wait_for(pilot, lambda: screen.outcome is not None and not screen.comparison_busy)
        comparison = screen.comparison
        assert comparison.nominal, text(screen.query_one('#comparison-status', Static))
        assert comparison.events
        assert 'no rail/noise' in text(screen.query_one('#comparison-status', Static))
        first = comparison.actual[0]
        baseline = comparison.nominal[0]
        for axis in 'xyz':
            assert float(baseline[f'h{axis}']) == float(first[f'h{axis}'])
        # The independent rollout starts with the same policy observation (CSV rounding aside).
        for i in range(3):
            assert float(baseline[f'ud{i}']) == pytest.approx(float(first[f'ud{i}']), abs=2e-4)
        # First predicted filtered position agrees with the deployed reference. The second
        # position uses the first filtered velocity, catching wrong lag state / force scaling.
        by_step = {int(row['k']): row for row in comparison.actual}
        for k, points in comparison.events.items():
            actual = by_step[k]
            for axis in 'xyz':
                assert float(points[0][f'raw_{axis}']) == float(actual[f'h{axis}'])
                assert float(points[1][f'filtered_{axis}']) == pytest.approx(float(actual[f'n{axis}']), abs=2e-6)
                expected = float(actual[f'n{axis}']) + float(actual[f'nv{axis}'])
                assert float(points[2][f'filtered_{axis}']) == pytest.approx(expected, abs=4e-6)
        # Select a different intervention and confirm both local branches are available.
        selector = screen.query_one('#comparison-event', Select)
        selector.value = min(comparison.events)
        await pilot.pause()
        for plane in ('xy', 'xz'):
            local = screen.query_one(f'#local-{plane}', PlotView)
            assert {trace.style for trace in local._overlays} == {RAW, FILTERED}
            assert local._xs[0] == float(by_step[min(comparison.events)]['hx'])
        output = screen.experiment.parent / 'comparisons' / screen.experiment.name
        assert Comparison.load(output, screen.experiment).events.keys() == comparison.events.keys()
        before = (output / 'nominal.csv').read_bytes()
        result = subprocess.run([str(app.project.build.rail_replay), '--preview', str(screen.experiment),
                                 str(app.project.policy.directory), str(output)], capture_output=True)
        assert result.returncode != 0
        assert (output / 'nominal.csv').read_bytes() == before
        # Generating a preview leaves the original recording's integrity/replay intact.
        result = subprocess.run([str(app.project.build.rail_replay), str(screen.experiment),
                                 str(app.project.policy.directory)], capture_output=True)
        assert result.returncode == 0, result.stderr
        screen.reset(tmp_path / 'next-run')
        assert not screen.comparison.nominal and not screen.comparison.events
        assert not screen.query_one('#plot-xy', PlotView)._overlays
        assert 'no data' in screen.query_one('#local-xy', PlotView).render().plain
