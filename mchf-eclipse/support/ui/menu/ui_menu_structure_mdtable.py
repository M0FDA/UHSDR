#!/usr/bin/python

"""
2016-12-24 HB9ocq, DB4ple - support program to document the menu structure of mcHF amateur radio SDR TRX

relies upon module  ui_menu_structure.py  in the same directory (generated by build of mcHF FW)


"""


from ui_menu_structure import MENU_DESCRIPTOR
# MENU_DESCRIPTOR is a list of dicts with entries "MENU_ID" "ME_KIND" "NR" "ID" "LABEL" "DESC"
# e.g.  [ ... { 'MENU_ID': "TOP",
#               'ME_KIND': "GROUP",
#               'NR': "MENU_BASE",
#               'ID': "STD",
#               'LABEL': "Standard Menu",
#               'DESC': ":soon:"},
#         ...]
        


from datetime import datetime

TS_NOW = datetime.now().replace(microsecond=0).isoformat()


BUILD_ID = "<${BUILD_ID}>"


#-----------------------------------------------------


# preface/header
print(r"""
[//]: # (                                                                              )
[//]: # ( WARNING: generated data!  DO NOT EDIT MANUALLY ! ! !                         )
[//]: # (                                                                              )
[//]: # ( {GEN_SENTENCE} )
[//]: # (                                                                              )
[//]: # ( mcHF SDR TRX - Menu Structure Diagram as MarkDown-Table                      )
[//]: # (                                                                              )
[//]: # ( see <https://help.github.com/categories/writing-on-github/>                  )
[//]: # (                                                                              )
    """.format(
        GEN_SENTENCE="generated from  {0}  at  {1}  by  ui_menu_structure_mdtable.py".format(BUILD_ID, TS_NOW)
    )
)

# all unique MENU_IDs
am = set([md['MENU_ID'] for md in MENU_DESCRIPTOR])

# a paragraph for every group
for gm in [md for md in MENU_DESCRIPTOR if (('MEK_GROUP' == md['ME_KIND']) and (md['NR'] in am))]:
    # header  H2
    print("""
## {LABEL} ({ID}, `{NR}`)""".format(**gm))

    # table header
    print("""
| {:<25}     ({:>3}) | {:<46} | """.format("LABEL", "ID", "DESCRIPTION"))
    print("""\
| {0:-<25}------{0:->3}- | {0:-<46} | """.format("-"))
    # table rows
    for md in MENU_DESCRIPTOR:
        if((0 != md['NR']) and ('MEK_STOP' != md['ME_KIND']) and (gm['NR'] ==  md['MENU_ID'])):
            # for sensible entries only
            md['LABEL'] = "**{LABEL}**".format(**md)
            print("""\
| {LABEL:<29} ({ID:>3}) | {DESC:<46} | """.format(**md))

    print("""""")
    
# footer
print(r"""
[//]: # ( EOFILE                                                                       )
""")

#EOFILE