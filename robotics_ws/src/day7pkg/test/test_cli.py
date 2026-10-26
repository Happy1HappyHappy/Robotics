"""
Tests of the two command-line tools on synthetic grid.npz files (no ROS, no robot), from this
package's directory: PYTHONPATH=../day6pkg:. python3 -m pytest test
"""

from day7pkg import find_landmarks, plan_path
import synthetic


def test_landmarks_writes_the_images_and_the_report(tmp_path):
    grid_a, grid_b = synthetic.save_pair(str(tmp_path))
    out = tmp_path / 'out'
    assert find_landmarks.main([grid_a, grid_b, '--output-dir', str(out)]) == 0
    for name in ('landmarks_a.png', 'landmarks_b.png', 'landmarks_overlay.png'):
        assert (out / name).stat().st_size > 0
    text = (out / 'landmarks.txt').read_text()
    assert 'Odometry:  ' in text and 'Landmarks: ' in text
    assert 'Mean difference in position' in text
    assert 'Only in A:' in text and 'Only in B:' in text


def test_plan_writes_the_path_and_refuses_a_blocked_goal(tmp_path, capsys):
    grid_a, _ = synthetic.save_pair(str(tmp_path))
    out = tmp_path / 'out'
    assert plan_path.main([grid_a, '--goal', '2.2', '0.7', '--output-dir', str(out)]) == 0
    assert (out / 'path.png').stat().st_size > 0
    text = (out / 'path.txt').read_text()
    assert 'blocked, so the path has to go around' in text
    assert '  t ' in text and '  m ' in text
    assert plan_path.main([grid_a, '--goal', '1.45', '0.75', '--output-dir', str(out)]) == 1
    assert 'blocked' in capsys.readouterr().out
