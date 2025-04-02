
##################################################################################################################

try:
    from maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.datastructures.organisation.Organisation import Organisation

except ImportError:
    from maaf_tools.maaf_tools.datastructures.agent.Agent import Agent
    from maaf_tools.maaf_tools.datastructures.organisation.Organisation import Organisation

##################################################################################################################


def agent_config_loader(path: str) -> Agent:
    # ----- Load organisation model
    organisation = Organisation(path)