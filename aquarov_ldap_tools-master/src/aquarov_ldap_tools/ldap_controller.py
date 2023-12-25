import base64
import hashlib

import ldap
from ldap.modlist import addModlist


# Parameters
BASE_DN = "dc=ldap,dc=aquarov,dc=cl"


class LdapController:

    def __init__(self, ldap_server_url):
        self.ldap_server_url = ldap_server_url
        self.ldap_base_dn = BASE_DN
        self.ldap = ldap.initialize("ldap://dummy")

    def __enter__(self):

        print(f"Connecting to LDAP server {self.ldap_server_url}...")

        self.ldap = ldap.initialize(self.ldap_server_url)
        self.ldap.simple_bind_s()  # ping ldap server before continuing

        print("Connected...\n")

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ldap.unbind_s()

    def _get_user_dn(self, user_uid):
        return "uid=" + user_uid + f",ou=users,{self.ldap_base_dn}"

    def _max_uid_number(self):
        # Search for all users with uidNumber attribute
        base_dn = f"ou=users,{self.ldap_base_dn}"
        search_filter = "(uidNumber=*)"
        attrs = ["uidNumber"]
        result = self.ldap.search_s(base_dn, ldap.SCOPE_SUBTREE, search_filter, attrs)

        # Find the highest uidNumber value
        return max([int(r[1]["uidNumber"][0]) for r in result])

    def log_as_admin(self, admin_pass):
        self.ldap.simple_bind_s(f"cn=admin,{self.ldap_base_dn}", admin_pass)

    def log_as_user(self, username, user_pass):
        self.ldap.simple_bind_s(f"uid={username},ou=users," + self.ldap_base_dn, user_pass)

    def list_users(self):
        # Search for users in the ou=users organizational unit
        base_dn = f"ou=users,{self.ldap_base_dn}"
        search_filter = "(objectClass=posixAccount)"
        attrs = ["uidNumber", "*"]
        result = self.ldap.search_s(base_dn, ldap.SCOPE_SUBTREE, search_filter, attrs)

        # Sort the search results by uidNumber attribute, if it exists
        result.sort(key=lambda x: int(x[1].get("uidNumber", [999999])[0]))

        # Convert byte values to unicode strings
        for r in result:
            for attr in r[1]:
                if isinstance(r[1][attr][0], bytes):
                    r[1][attr][0] = r[1][attr][0].decode()

        return result

    def create_user(self,
                    uid,
                    first_name,
                    second_name,
                    email,
                    hashed_password_b64):

        # Create a new uid number
        uid_number = f"{self._max_uid_number() + 1}"

        # Define the new user's attributes
        new_user_dn = f"uid={uid},ou=users,dc=ldap,dc=aquarov,dc=cl"

        new_user_attrs = dict(
            objectClass=[b"inetOrgPerson", b"posixAccount", b"shadowAccount"],
            cn=f"{first_name} {second_name}".encode(),
            givenName=first_name.encode(),
            sn=second_name.encode(),
            employeeType=b"developer",
            uidNumber=uid_number.encode(),
            gidNumber=uid_number.encode(),
            mail=email.encode(),
            homeDirectory=f"/home/{uid}".encode(),
            loginShell=b"/bin/bash",
            userPassword=hashed_password_b64.encode(),
        )

        # Add the new user's entry to the LDAP directory
        self.ldap.add_s(new_user_dn, addModlist(new_user_attrs))

    def delete_user(self, user_uid):
        user_dn = self._get_user_dn(user_uid)
        self.ldap.delete_s(user_dn)

    def change_user_password(self, user_uid, password):
        user_dn = self._get_user_dn(user_uid)

        hashed_password = hashlib.sha1(password.encode()).digest()
        hashed_password_b64 = "{SHA}" + base64.b64encode(hashed_password).decode()

        # Modificación del atributo "userPassword"
        mod_attrs = [(ldap.MOD_REPLACE, "userPassword", hashed_password_b64.encode())]
        self.ldap.modify_s(user_dn, mod_attrs)