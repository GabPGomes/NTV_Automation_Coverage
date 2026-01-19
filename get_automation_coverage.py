from pyverilog.vparser.ast import (
    ModuleDef, InstanceList, Instance, PortArg, 
    Decl, Assign, Always, NonblockingSubstitution, BlockingSubstitution
)
from pyverilog.vparser.parser import parse
import os
import argparse

def count_procedural_assignments(node):
    """
    Função auxiliar para contar atribuições dentro de blocos always.
    """
    count = 0
    if isinstance(node, (NonblockingSubstitution, BlockingSubstitution)):
        count += 1
    
    # Percorre os nós filhos (como blocos 'begin...end' ou 'if...else')
    for child in node.children():
        count += count_procedural_assignments(child)
    return count

def count_elements(ast):
    modules = 0
    instances = 0
    top_level_ports = 0
    ports_declared = 0
    ports_connected = 0
    wires = 0
    assigns = 0
    always_blocks = 0
    procedural_assigns = 0

    for desc in ast.description.definitions:
        if isinstance(desc, ModuleDef):
            modules += 1
            
            # Top-level interface
            if desc.portlist:
                top_level_ports += len(desc.portlist.ports)

            for item in desc.items:
                # Instâncias
                if isinstance(item, InstanceList):
                    for inst in item.instances:
                        if isinstance(inst, Instance):
                            instances += 1
                            for conn in inst.portlist:
                                if isinstance(conn, PortArg):
                                    ports_declared += 1
                                    if conn.argname is not None:
                                        ports_connected += 1

                # Wires/Regs
                elif isinstance(item, Decl):
                    wires += len(item.list)
                
                # Atribuições Contínuas (assign)
                elif isinstance(item, Assign):
                    assigns += 1
                
                # Blocos Always e atribuições internas
                elif isinstance(item, Always):
                    always_blocks += 1
                    procedural_assigns += count_procedural_assignments(item)

    return {
        "modules": modules,
        "instances": instances,
        "top_level_ports": top_level_ports,
        "ports_declared": ports_declared,
        "ports_connected": ports_connected,
        "wires": wires,
        "continuous_assigns": assigns,
        "always_blocks": always_blocks,
        "procedural_assigns": procedural_assigns
    }

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="Count Verilog elements")
    argparser.add_argument("core_basename", help="Basename of the core")
    args = argparser.parse_args()

    folders = {
        "empty": "empty_wrappers_verilog",
        "final": "final_wrappers_verilog"
    }
    
    results = {}

    for mode, folder in folders.items():
        verilog_file = os.path.abspath(f"{folder}/{args.core_basename}.v")
        if not os.path.isfile(verilog_file):
            print(f"Warning: File {verilog_file} not found. Skipping {mode}.")
            results[mode] = {}
            continue

        print(f"Scanning {mode} wrapper for {args.core_basename}...")
        ast, _ = parse(
            [verilog_file],
            preprocess_include=[folder],
            preprocess_define=["SIMULATION"]
        )
        results[mode] = count_elements(ast)
        print(results[mode])

    # Cálculo da cobertura
    sum_empty = sum(results["empty"].values()) if results["empty"] else 0
    sum_completed = sum(results["final"].values()) if results["final"] else 0
    
    coverage = (sum_empty / sum_completed * 100) if sum_completed != 0 else 0
    
    output = f"\nCoverage for {args.core_basename}: {coverage:.2f}%\n"
    print(output)
    
    with open("total.coverage", "a") as f:
        f.write(output)