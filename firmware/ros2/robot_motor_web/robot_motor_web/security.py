"""Anonymous browser sessions for control arbitration and same-origin CSRF, not login."""
import secrets,time,hmac
class Sessions:
    def __init__(self):self.sessions={}
    def session(self,token):
        s=self.sessions.get(token)
        if s and s['expires']>time.monotonic():return s
        self.sessions.pop(token,None);return None
    def create(self):
        t=secrets.token_urlsafe(32);c=secrets.token_urlsafe(32)
        self.sessions[t]=dict(csrf=c,expires=time.monotonic()+8*3600)
        return t,c
    def check_csrf(self,token,csrf):
        s=self.session(token)
        return bool(s and csrf and hmac.compare_digest(s['csrf'],csrf))
