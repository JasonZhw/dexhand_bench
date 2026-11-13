"""Compose the proposed task owner; never represent it as a physical run.

Use --cfg job --resolve for inspection. Normal invocation writes Hydra's
resolved job configuration and exits with a clear runtime-blocked error.
"""
import hydra
from omegaconf import DictConfig, OmegaConf


@hydra.main(version_base=None, config_path='../../conf/unilab', config_name='benchmark')
def main(cfg: DictConfig) -> None:
    print(OmegaConf.to_yaml(cfg, resolve=True))
    raise SystemExit(
        'CONFIGURATION PROTOTYPE ONLY: no physical UniLab task factory is registered. '
        'Resolve task.runtime.blockers before adding a simulation launcher. '
        'Use --cfg job --resolve to inspect configuration without launching.'
    )


if __name__ == '__main__':
    main()
