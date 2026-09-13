"""Compile the streaming-sink assignment from audio.cpp and pin its restore debt.

A session reclaimed on reconnect inherits restore_sink=true from the session that
hijacked Windows' default endpoint. Recomputing the flag from the selected sink
would clear that debt whenever the reclaimed session streams the host sink, and
stop_audio_control() would then never restore the default.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'src/audio.cpp').read_text()
start = source.index('        const bool sink_differs =')
guard = source.index('if (sink_differs)', start)
terminator = '\n        }\n'
end = source.index(terminator, guard) + len(terminator)
block = source[start:end]

program = '''
#include <cassert>
#include <string>

struct control_t {
  int set_sink_calls = 0;
  std::string last_sink;

  int set_sink(const std::string &sink) {
    ++set_sink_calls;
    last_sink = sink;
    return 0;  // success: the production block returns right after
  }
};

struct sink_t {
  std::string host;
};

struct ctx_t {
  sink_t sink;
  bool restore_sink;
};

void assign(ctx_t *ref, control_t *control, const std::string &sink) {
''' + block + '''
}

int main() {
  // Fresh session, streaming sink differs from the host sink: assign and owe a restore.
  {
    ctx_t ctx {{"host"}, false};
    control_t control;
    assign(&ctx, &control, "virtual");
    assert(ctx.restore_sink);
    assert(control.set_sink_calls == 1 && control.last_sink == "virtual");
  }

  // Fresh session on the host sink: unchanged default path, nothing to restore.
  {
    ctx_t ctx {{"host"}, false};
    control_t control;
    assign(&ctx, &control, "host");
    assert(!ctx.restore_sink);
    assert(control.set_sink_calls == 0);
  }

  // Reclaimed session that now selects the host sink: the inherited restore debt
  // must survive, and the host sink must not be re-asserted as a new assignment.
  {
    ctx_t ctx {{"host"}, true};
    control_t control;
    assign(&ctx, &control, "host");
    assert(ctx.restore_sink);
    assert(control.set_sink_calls == 0);
  }

  // Reclaimed session on a different sink: re-assert it and keep owing a restore.
  {
    ctx_t ctx {{"host"}, true};
    control_t control;
    assign(&ctx, &control, "virtual");
    assert(ctx.restore_sink);
    assert(control.set_sink_calls == 1);
  }
}
'''

with tempfile.TemporaryDirectory(prefix='audio-restore-sink-') as directory:
    path = Path(directory)
    (path / 'test.cpp').write_text(program)
    subprocess.run(['g++', '-std=c++17', str(path / 'test.cpp'), '-o', str(path / 'test')], check=True)
    subprocess.run([str(path / 'test')], check=True)
print('PASS: restore_sink latches across a reclaimed session; the default path is unchanged')
