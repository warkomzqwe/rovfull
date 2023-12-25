import base64
import hashlib

from tabulate import tabulate
from getpass import getpass
from threading import Event

from aquarov_ldap_tools.utils import clear_screen
from aquarov_ldap_tools.ldap_controller import LdapController


LDAP_SERVER = "ldap://server1.telemcloud.cl:389"


def pretty_user_list(user_list):
    # Format the search results into a table
    headers = ["UID", "First Name", "Last Name", "Email", "Number"]
    rows = [
        [r[1].get("uid", [""])[0], r[1].get("givenName", [""])[0], r[1].get("sn", [""])[0], r[1].get("mail", [""])[0],
         r[1].get("uidNumber")[0]] for r in user_list]

    return tabulate(rows, headers=headers, tablefmt="fancy_grid")


def new_user_routine(ldap_controller: LdapController):
    """User creation including a hashed password"""

    uid = input("New username: ")
    email = input("E-mail: ")
    first_name = input("First name: ")
    second_name = input("Second name: ")
    password = input("New password: ")
    hashed_password = hashlib.sha1(password.encode()).digest()
    hashed_password_b64 = "{SHA}" + base64.b64encode(hashed_password).decode()

    ldap_controller.create_user(uid=uid,
                                first_name=first_name,
                                second_name=second_name,
                                email=email,
                                hashed_password_b64=hashed_password_b64)


def delete_user_routine(ldap_controller: LdapController):
    """User delete routine"""

    uid = input("Delete user by uid: ")
    ldap_controller.delete_user(uid)


def main():
    clear_screen()
    print("Aquarov development LDAP administration TUI.\n")

    ldap_controller = LdapController(LDAP_SERVER)
    stop_loop_flag = Event()

    with ldap_controller:

        admin_pass = getpass("Enter admin password: ")
        ldap_controller.log_as_admin(admin_pass)

        while not stop_loop_flag.is_set():

            clear_screen()

            # Output all users
            print(pretty_user_list(ldap_controller.list_users()))

            input_ = input("Press 1 for create new user.\n"
                           "Press 2 for delete existing user.\n"
                           "Press 3 to refresh.\n"
                           "Press 4 to exit.\n"
                           "")

            input_ = int(input_)

            if input_ == 1:
                clear_screen()
                print("New user routine: \n")
                new_user_routine(ldap_controller)

            elif input_ == 2:
                print("Delete user routine: \n")
                delete_user_routine(ldap_controller)

            elif input_ == 3:
                clear_screen()
                continue

            elif input_ == 4:
                stop_loop_flag.set()

            else:
                pass


if __name__ == '__main__':
    main()