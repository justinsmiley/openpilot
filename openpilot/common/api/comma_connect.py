import os

import requests

from openpilot.common.api.base import BaseApi
from openpilot.system.smiley_privacy import BLOCK_COMMA_CLOUD

API_HOST = os.getenv('API_HOST', 'https://api.commadotai.com')


class CommaConnectApi(BaseApi):
  def __init__(self, dongle_id):
    super().__init__(dongle_id, API_HOST)
    self.user_agent = "openpilot-"

  def api_get(self, endpoint, *args, **kwargs):
    # smileypilot privacy switch: every request to comma's API goes through here, so one guard covers
    # prime status, trips, GPS assist, registration, uploads... Callers see an ordinary network error.
    if BLOCK_COMMA_CLOUD:
      raise requests.exceptions.ConnectionError("comma cloud blocked (smileypilot)")
    return super().api_get(endpoint, *args, **kwargs)
